# -*- coding: utf-8 -*-
"""Công cụ của người viết code: kiểm tra bộ dữ liệu nhóm train bàn giao (chỉ đọc)."""
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

ROOT = Path(__file__).resolve().parent.parent
TOOL = ROOT / "training" / "check_dataset.py"


def run(path):
    r = subprocess.run([sys.executable, str(TOOL), str(path)], capture_output=True, text=True, cwd=ROOT)
    return r.returncode, r.stdout


def make(root: Path, layout="ultralytics", names=("tram_1", "tram_2", "tram_3"), sessions=None, w=640, h=480):
    rng = np.random.default_rng(1)
    sessions = sessions or {"train": ["A", "B", "C"], "val": ["D"], "test": ["E"]}
    for split, sess in sessions.items():
        d = "valid" if (layout == "roboflow" and split == "val") else split
        img_dir = root / ("images/" + d if layout == "ultralytics" else d + "/images")
        lab_dir = root / ("labels/" + d if layout == "ultralytics" else d + "/labels")
        img_dir.mkdir(parents=True, exist_ok=True)
        lab_dir.mkdir(parents=True, exist_ok=True)
        for s in sess:
            for i in range(6):
                name = f"{s}_{i:02d}"
                cv2.imwrite(str(img_dir / f"{name}.jpg"), rng.integers(0, 255, (h, w, 3), dtype=np.uint8))
                (lab_dir / f"{name}.txt").write_text(f"{i % 3} 0.5 0.5 0.2 0.3\n")
    if layout == "ultralytics":
        y = "path: .\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n" + \
            "".join(f"  {i}: {n}\n" for i, n in enumerate(names))
    else:
        y = f"train: ../train/images\nval: ../valid/images\ntest: ../test/images\nnc: {len(names)}\nnames: {list(names)}\n"
    (root / "data.yaml").write_text(y)
    return root


def test_good_dataset_passes(tmp_path):
    code, out = run(make(tmp_path / "ds"))
    assert code == 0, out
    assert "ĐẠT" in out and "[LỖI]" not in out


def test_roboflow_zip_with_wrong_class_order_fails(tmp_path):
    ds = make(tmp_path / "rf", layout="roboflow", names=("tram_2", "tram_1", "tram_3"))
    z = tmp_path / "rf.zip"
    with zipfile.ZipFile(z, "w") as f:
        for p in ds.rglob("*"):
            f.write(p, p.relative_to(ds))
    code, out = run(z)
    assert code == 1 and "KHÁC MODEL_CLASSES" in out
    assert "val (valid/images)" in out                       # nhận ra kiểu thư mục Roboflow


def test_bad_label_line_fails(tmp_path):
    ds = make(tmp_path / "ds")
    (ds / "labels" / "train" / "A_00.txt").write_text("5 0.5 0.5 0.2 0.2\n0 1.4 0.5 0.1 0.1\n")
    code, out = run(ds)
    assert code == 1 and "mã lớp 5" in out and "tọa độ ngoài" in out


def test_session_leak_and_duplicates_are_warned(tmp_path):
    ds = make(tmp_path / "ds", sessions={"train": ["A", "B"], "val": ["A"], "test": ["C"]})
    shutil.copy(ds / "images" / "train" / "B_00.jpg", ds / "images" / "test" / "C_99.jpg")
    code, out = run(ds)
    assert "nằm ở nhiều tập" in out and "GẦN TRÙNG" in out
    assert code == 0                                           # chỉ cảnh báo, không phải lỗi


def test_missing_class_in_split_and_wrong_aspect(tmp_path):
    ds = make(tmp_path / "ds", w=640, h=360)
    for p in (ds / "labels" / "test").glob("*.txt"):
        p.write_text("0 0.5 0.5 0.2 0.2\n")                   # tập test chỉ còn tram_1
    code, out = run(ds)
    assert code == 1 and "tập test KHÔNG có đối tượng lớp 1" in out
    assert "4:3" in out


def test_eval_reads_roboflow_layout(tmp_path, monkeypatch):
    """test_model --eval phải đọc đúng tập test kiểu Roboflow (test/images), không lẫn ảnh train."""
    sys.path.insert(0, str(ROOT / "training"))
    import test_model as tm
    monkeypatch.setattr(tm.cfg, "LOG_DIR", str(tmp_path / "logs"))
    ds = make(tmp_path / "rf", layout="roboflow")

    class Perfect:
        version, names, last_ms = "x", {0: "tram_1", 1: "tram_2", 2: "tram_3"}, 1.0
        seen = 0

        def detect(self, frame):
            Perfect.seen += 1
            return []
    stat, rows = tm.evaluate(Perfect(), ds, "test")
    assert Perfect.seen == 6                                   # chỉ 6 ảnh của tập test
    assert sum(s["bo_sot"] for s in stat.values()) == 6
