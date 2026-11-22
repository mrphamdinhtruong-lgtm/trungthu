# -*- coding: utf-8 -*-
"""Giao tiếp Pi–Arduino (Arduino giả qua pty), nguồn ảnh/video, chia dữ liệu theo buổi, đánh giá mô hình."""
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from robot import config, fwconfig  # noqa: E402
from robot.common import Detection  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


@pytest.mark.skipif(os.name != "posix", reason="cần pty (Linux/macOS)")
def test_serial_link_with_fake_arduino_timeout_estop_and_unplug():
    pytest.importorskip("serial")
    from robot.fake_arduino import FakeArduino
    from robot.serial_link import SerialLink
    fa = FakeArduino(config, fwconfig.config_hash(config), drop_after_s=9.0).start()
    link = SerialLink(fa.port, config.SERIAL_BAUD).open(wait_info_s=3.0)
    try:
        assert link.info and link.info["hash"] == fwconfig.config_hash(config)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:                  # gửi lệnh đều 20 Hz
            link.send_wheels([40, 40, 40, 40])
            time.sleep(0.05)
        assert link.alive() and link.telemetry.rpm[0] > 30 and not link.telemetry.cmd_timeout
        time.sleep(1.0)                                     # NGỪNG gửi lệnh -> Arduino tự dừng
        assert link.telemetry.cmd_timeout and abs(link.telemetry.rpm[0]) < 5
        link.estop(True)
        time.sleep(0.3)
        assert link.telemetry.estop
        link.estop(False)
        time.sleep(0.3)
        assert not link.telemetry.estop
        while time.monotonic() - t0 < 10.5:                 # giả rút cáp ở giây thứ 9
            time.sleep(0.2)
        assert not link.alive(config.LINK_TIMEOUT_S)
    finally:
        link.close()
        fa.stop()


def test_camera_from_image_folder_and_video(tmp_path):
    from robot.camera import Camera
    for i in range(3):
        cv2.imwrite(str(tmp_path / f"a_{i}.jpg"), np.full((300, 400, 3), 40 * i, np.uint8))
    cam = Camera(width=640, height=480, video_path=str(tmp_path), image_fps=20).start()
    time.sleep(0.5)
    f, t = cam.read()
    cam.stop()
    assert f is not None and f.shape == (480, 640, 3) and cam.frame_count >= 3
    vid = tmp_path / "v.avi"
    vw = cv2.VideoWriter(str(vid), cv2.VideoWriter_fourcc(*"MJPG"), 10, (640, 480))
    for i in range(5):
        vw.write(np.full((480, 640, 3), 30 * i, np.uint8))
    vw.release()
    cam = Camera(video_path=str(vid)).start()
    time.sleep(1.2)
    f, _ = cam.read()
    n = cam.frame_count
    cam.stop()
    assert f is not None and n > 5                          # phát lặp lại khi hết video


def _make_dataset(root: Path):
    rng = np.random.default_rng(0)
    for s, sess in enumerate(["A_sang", "A_chieu", "B_sang", "B_chieu", "C_sang"]):
        d = root / sess
        d.mkdir(parents=True)
        for i in range(10):
            img = rng.integers(0, 255, (120, 160, 3), dtype=np.uint8)
            cv2.imwrite(str(d / f"{sess}_{i}.jpg"), img)
            (d / f"{sess}_{i}.txt").write_text(f"{i % 3} 0.5 0.5 0.2 0.2\n")
    cv2.imwrite(str(root / "C_sang" / "dup.jpg"), cv2.imread(str(root / "A_sang" / "A_sang_0.jpg")))
    (root / "B_sang" / "B_sang_0.txt").write_text("7 0.5 0.5 0.2 0.2\n")       # mã lớp sai


def test_split_by_session_detects_duplicates_and_bad_labels(tmp_path):
    src, dst = tmp_path / "labeled", tmp_path / "ds"
    _make_dataset(src)
    r = subprocess.run([sys.executable, str(ROOT / "training" / "split_dataset.py"), "--src", str(src),
                        "--dst", str(dst), "--val", "0.2", "--test", "0.2"], capture_output=True, text=True)
    out = r.stdout
    assert r.returncode == 1 and "SAI định dạng" in out           # dòng nhãn lớp 7 bị bắt
    assert "GẦN TRÙNG" in out
    sessions = {}
    for split in ("train", "val", "test"):
        files = list((dst / "images" / split).glob("*.jpg"))
        assert files, split
        for f in files:
            sessions.setdefault(f.name.split("__")[0], set()).add(split)
    assert all(len(v) == 1 for v in sessions.values()), sessions  # mỗi buổi chỉ thuộc 1 tập


def test_evaluate_counts_correct_wrong_missed_false(tmp_path, monkeypatch):
    sys.path.insert(0, str(ROOT / "training"))
    import test_model as tm
    monkeypatch.setattr(tm.cfg, "LOG_DIR", str(tmp_path / "logs"))
    img_dir, lab_dir = tmp_path / "images" / "test", tmp_path / "labels" / "test"
    img_dir.mkdir(parents=True)
    lab_dir.mkdir(parents=True)
    for i in range(4):
        cv2.imwrite(str(img_dir / f"{i}.jpg"), np.zeros((48, 64, 3), np.uint8))
        (lab_dir / f"{i}.txt").write_text(f"{i % 3} 0.5 0.5 0.2 0.2\n")

    class Fake:
        version, names, last_ms = "fake", {0: "tram_1", 1: "tram_2", 2: "tram_3"}, 1.0
        calls = 0

        def detect(self, frame):
            k = Fake.calls
            Fake.calls += 1
            box = (0.4, 0.4, 0.6, 0.6)
            if k == 0:
                return [Detection("tram_1", 0.9, *box)]                   # đúng
            if k == 1:
                return [Detection("tram_3", 0.8, *box)]                   # nhầm trạm (thật: tram_2)
            if k == 2:
                return []                                                 # bỏ sót tram_3
            return [Detection("tram_1", 0.9, *box), Detection("tram_2", 0.7, 0.0, 0.0, 0.1, 0.1)]  # + báo giả
    stat, rows = tm.evaluate(Fake(), tmp_path, "test")
    assert stat["tram_1"]["dung"] == 2 and stat["tram_2"]["nham"] == 1 and stat["tram_3"]["bo_sot"] == 1
    assert stat["tram_2"]["bao_gia"] == 1
    assert {r[1] for r in rows} == {"NHẦM TRẠM", "BỎ SÓT", "BÁO GIẢ"}
