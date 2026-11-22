#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NHẬN best.pt TỪ NHÓM TRAIN -> KIỂM TRA LỚP -> XUẤT best.onnx + best.metadata.json

  python training/export_model.py --weights best.pt --version tram3_v1
  python training/export_model.py --weights best.pt --version tram3_v1 --args args.yaml --results results.csv
  python training/export_model.py --check models/best.onnx      # chỉ kiểm tra lại file đã xuất

Kết quả (đặt MODEL_PATH = "models/best.onnx" trong robot/config.py):
  models/best.onnx            mô hình ONNX ảnh tĩnh (mặc định 416), không gộp NMS (code tự lọc)
  models/best.metadata.json   phiên bản, tên & thứ tự lớp, kích thước ảnh, mã băm, cấu hình train
  models/best.pt              bản sao trọng số gốc (để truy vết)

Bước KIỂM TRA LỚP: tên + thứ tự lớp trong .pt phải đúng robot/config.py MODEL_CLASSES
(kế hoạch: 0=tram_1, 1=tram_2, 2=tram_3). Sai -> dừng, KHÔNG xuất (tránh robot đi nhầm trạm).
Cần: pip install ultralytics onnx  (chạy trên laptop hoặc Pi đều được; Pi chỉ cần onnxruntime để CHẠY).
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_args_yaml(path: Path) -> dict:
    try:
        import yaml
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        out = {}
        for line in path.read_text(encoding="utf-8").splitlines():
            if ":" in line and not line.startswith(" "):
                k, v = line.split(":", 1)
                out[k.strip()] = v.strip()
        return out


def read_results_csv(path: Path) -> dict:
    """Dòng cuối results.csv của Ultralytics -> các chỉ số mAP / precision / recall."""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    last = {k.strip(): v.strip() for k, v in rows[-1].items() if k}
    keep = {}
    for k, v in last.items():
        if k.startswith("metrics/") or k == "epoch":
            try:
                keep[k] = float(v)
            except ValueError:
                pass
    return keep


def check_names(names: dict, expected) -> list:
    got = [names[i] for i in sorted(names)]
    errs = []
    if list(expected) and got != list(expected):
        errs.append(f"Tên/thứ tự lớp trong mô hình {got} KHÁC config MODEL_CLASSES {list(expected)}")
    return errs


def export(weights, imgsz: int = 416, version: str = "", args_yaml=None, results_csv=None,
           out_dir: Path = PROJECT_DIR / "models", opset: int = 12) -> Path:
    from ultralytics import YOLO
    import ultralytics
    weights = Path(weights)
    model = YOLO(str(weights))
    names = {int(k): str(v) for k, v in model.names.items()}
    errs = check_names(names, getattr(cfg, "MODEL_CLASSES", ()))
    if errs:
        raise SystemExit("[export] DỪNG: " + "; ".join(errs) +
                         "\n  -> Nhóm train phải train lại đúng thứ tự lớp, hoặc sửa MODEL_CLASSES nếu cố ý đổi.")
    print(f"[export] Lớp OK: {names}")
    onnx_tmp = Path(model.export(format="onnx", imgsz=imgsz, opset=opset, simplify=True, dynamic=False,
                                 nms=False))
    out_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = out_dir / "best.onnx"
    shutil.copy2(onnx_tmp, onnx_path)
    pt_copy = out_dir / "best.pt"
    if weights.resolve() != pt_copy.resolve():
        shutil.copy2(weights, pt_copy)
    meta = {
        "model_version": version or time.strftime("tram3_%Y%m%d_%H%M"),
        "format": "onnx",
        "model_file": onnx_path.name,
        "task": "detect",
        "names": {str(k): v for k, v in names.items()},
        "nc": len(names),
        "imgsz": [imgsz, imgsz],
        "input": "RGB, 0..1, NCHW, letterbox (114)",
        "output": "(1, 4+nc, N): cx, cy, w, h, điểm từng lớp – chưa NMS",
        "source_weights": weights.name,
        "sha256_pt": sha256(weights),
        "sha256_onnx": sha256(onnx_path),
        "exported_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "ultralytics": getattr(ultralytics, "__version__", "?"),
        "opset": opset,
    }
    a = Path(args_yaml) if args_yaml else weights.parent.parent / "args.yaml"
    if a.exists():
        tr = read_args_yaml(a)
        meta["train_args"] = {k: tr.get(k) for k in ("model", "epochs", "imgsz", "batch", "data", "patience",
                                                     "fliplr", "mosaic") if k in tr}
    r = Path(results_csv) if results_csv else weights.parent.parent / "results.csv"
    if r.exists():
        meta["metrics"] = read_results_csv(r)
    mp = out_dir / "best.metadata.json"
    mp.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[export] Đã tạo {onnx_path} và {mp}")
    print(f"[export] Phiên bản: {meta['model_version']}  |  ảnh {imgsz}  |  {meta['nc']} lớp")
    print('[export] robot/config.py: MODEL_PATH = "models/best.onnx"')
    return onnx_path


def check(onnx_path: Path) -> int:
    """Kiểm tra file đã xuất: đọc metadata, chạy thử 1 ảnh đen, so lớp với config."""
    import numpy as np
    from robot.detector import OnnxDetector, check_classes
    det = OnnxDetector(str(onnx_path))
    errors, notes = check_classes(det.names, cfg)
    det.detect(np.zeros((cfg.FRAME_HEIGHT, cfg.FRAME_WIDTH, 3), np.uint8))
    print(f"[check] {onnx_path.name}: phiên bản {det.version}, lớp {det.names}, ảnh {det.shape}, "
          f"{det.backend}, {det.last_ms:.0f} ms/khung")
    for n in notes:
        print("[check] lưu ý:", n)
    for e in errors:
        print("[check] LỖI:", e)
    if det.meta.get("sha256_onnx"):
        ok = sha256(onnx_path) == det.meta["sha256_onnx"]
        print("[check] mã băm ONNX", "khớp metadata" if ok else "KHÔNG khớp metadata (file bị thay?)")
        if not ok:
            errors.append("sha256")
    return 1 if errors else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", help="best.pt nhóm train bàn giao")
    ap.add_argument("--imgsz", type=int, default=cfg.YOLO_IMGSZ)
    ap.add_argument("--version", default="", help="tên phiên bản mô hình, vd tram3_v1")
    ap.add_argument("--args", default=None, help="args.yaml của lần train (nếu có)")
    ap.add_argument("--results", default=None, help="results.csv của lần train (nếu có)")
    ap.add_argument("--check", default=None, help="chỉ kiểm tra một file .onnx đã xuất")
    ap.add_argument("--out", default=str(PROJECT_DIR / "models"), help="thư mục xuất (mặc định models/)")
    a = ap.parse_args()
    if a.check:
        sys.exit(check(Path(a.check)))
    if not a.weights:
        ap.error("cần --weights best.pt hoặc --check models/best.onnx")
    path = export(a.weights, a.imgsz, a.version, a.args, a.results, out_dir=Path(a.out))
    sys.exit(check(path))


if __name__ == "__main__":
    main()
