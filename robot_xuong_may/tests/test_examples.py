# -*- coding: utf-8 -*-
"""Các bài tập cho người mới (examples/) phải luôn chạy được."""
import os
import sys
from pathlib import Path

import pytest

from conftest import run_script

ROOT = Path(__file__).resolve().parent.parent
EX = ROOT / "examples"


def run(*args, cwd=None):
    return run_script(args, cwd=cwd, timeout=120)


def test_bai4_state_machine():
    r = run(EX / "04_may_trang_thai.py")
    assert r.returncode == 0, r.stderr
    assert "ĐÃ ĐẾN TRẠM 1" in r.stdout and "ĐANG DI CHUYỂN ĐẾN TRẠM 2" in r.stdout
    assert "còn 15 s" in r.stdout


@pytest.mark.skipif(os.name != "posix", reason="Arduino giả cần pty")
def test_bai3_fake_arduino():
    pytest.importorskip("serial")
    r = run(EX / "03_gui_lenh_arduino.py", "--fake")
    assert r.returncode == 0, r.stderr
    assert "cờ mất lệnh: True" in r.stdout


def test_bai1_video(tmp_path):
    cv2 = pytest.importorskip("cv2")
    import numpy as np
    vid = tmp_path / "v.avi"
    vw = cv2.VideoWriter(str(vid), cv2.VideoWriter_fourcc(*"MJPG"), 10, (640, 480))
    for i in range(10):
        vw.write(np.full((480, 640, 3), 20 * i, np.uint8))
    vw.release()
    r = run(EX / "01_xem_camera.py", "--video", vid, "--frames", 5, cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    assert "(480, 640, 3)" in r.stdout and (tmp_path / "bai1_khung_cuoi.jpg").exists()


def test_bai2_detect_with_fake_onnx(tmp_path):
    pytest.importorskip("onnx")
    pytest.importorskip("onnxruntime")
    cv2 = pytest.importorskip("cv2")
    import json
    import numpy as np
    sys.path.insert(0, str(ROOT / "tests"))
    from test_detector_onnx import build_model
    build_model(tmp_path / "best.onnx")
    (tmp_path / "best.metadata.json").write_text(json.dumps(
        {"model_version": "thu", "names": {"0": "tram_1", "1": "tram_2", "2": "tram_3"}, "imgsz": [416, 416]}))
    img = tmp_path / "a.jpg"
    cv2.imwrite(str(img), np.zeros((480, 640, 3), np.uint8))
    r = run(EX / "02_nhan_dien_anh.py", img, "--model", tmp_path / "best.onnx", cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    assert "tram_2" in r.stdout and (tmp_path / "bai2_ket_qua.jpg").exists()


@pytest.mark.parametrize("script", ["main.py", "flash_arduino.py", "tools/motor_test.py", "tools/fake_arduino.py",
                                    "training/check_dataset.py", "training/export_model.py", "training/test_model.py",
                                    "training/split_dataset.py", "training/capture_images.py"])
def test_cli_help_prints_vietnamese_on_windows_codepage(script):
    """Mọi lệnh phải in được tiếng Việt kể cả khi Windows dùng bảng mã cp1252 (kết quả bị chuyển hướng)."""
    r = run(ROOT / script, "--help")
    assert "UnicodeEncodeError" not in r.stderr, r.stderr
    assert r.returncode == 0, r.stderr[-500:]
