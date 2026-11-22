#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XUẤT MÔ HÌNH CHO RASPBERRY PI 4

  python training/export_model.py --weights runs/detect/tram_xuong_may/weights/best.pt
  -> models/best_ncnn_model/   (đặt MODEL_PATH = "models/best_ncnn_model" trong robot/config.py)

NCNN là định dạng nhanh nhất cho CPU ARM của Pi 4 (Ultralytics khuyến nghị).
Có thể chạy trên laptop rồi chép thư mục models/ sang Pi, hoặc chạy ngay trên Pi.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent


def export(weights, imgsz: int = 320, fmt: str = "ncnn") -> Path:
    from ultralytics import YOLO
    weights = Path(weights)
    model = YOLO(str(weights))
    out = Path(model.export(format=fmt, imgsz=imgsz))
    models_dir = PROJECT_DIR / "models"
    models_dir.mkdir(exist_ok=True)
    target = models_dir / (f"best_{fmt}_model" if out.is_dir() else f"best{out.suffix}")
    if target.exists():
        shutil.rmtree(target) if target.is_dir() else target.unlink()
    shutil.copytree(out, target) if out.is_dir() else shutil.copy2(out, target)
    shutil.copy2(weights, models_dir / "best.pt")
    print(f"Đã xuất: {target}")
    print(f'-> robot/config.py: MODEL_PATH = "models/{target.name}"')
    return target


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--format", default="ncnn", choices=["ncnn", "onnx", "openvino", "tflite"])
    args = ap.parse_args()
    export(args.weights, args.imgsz, args.format)


if __name__ == "__main__":
    main()
