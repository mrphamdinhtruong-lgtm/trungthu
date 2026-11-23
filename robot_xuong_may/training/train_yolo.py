#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HUẤN LUYỆN YOLO (chạy trên LAPTOP có GPU hoặc Google Colab – KHÔNG train trên Pi 4)

  pip install -r requirements-train.txt
  python training/train_yolo.py                          # theo kế hoạch: YOLO11n, 50 epochs, ảnh 416, batch Auto
  python training/train_yolo.py --epochs 80 --imgsz 416    # train lại lâu hơn nếu còn thời gian
  python training/train_yolo.py --export                 # train xong xuất luôn NCNN cho Pi

Kết quả: runs/detect/<tên>/weights/best.pt  (+ đồ thị mAP, confusion matrix cho báo cáo)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent

for _s in (sys.stdout, sys.stderr):          # Windows: in tiếng Việt khi bị chuyển hướng (cp1252)
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError, OSError):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(PROJECT_DIR / "training" / "data.yaml"))
    ap.add_argument("--model", default="yolo11n.pt", help="yolo11n.pt / yolov8n.pt (bản nano cho Pi 4)")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=416, help="NÊN bằng YOLO_IMGSZ dùng trên robot")
    ap.add_argument("--batch", type=int, default=-1, help="-1 = Auto (tự chọn theo bộ nhớ GPU)")
    ap.add_argument("--device", default=None, help="0 = GPU, cpu = CPU (mặc định tự chọn)")
    ap.add_argument("--name", default="tram3")
    ap.add_argument("--export", action="store_true", help="xuất NCNN sau khi train")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("Chưa cài ultralytics: pip install -r requirements-train.txt")

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(PROJECT_DIR / "runs" / "detect"),
        name=args.name,
        patience=30,
        # ---- tăng cường dữ liệu phù hợp biển báo có CHỮ SỐ ----
        fliplr=0.0,          # KHÔNG lật ngang: số "1/2/3" bị lật sẽ sai hình dạng
        flipud=0.0,
        degrees=5.0,         # xoay nhẹ (robot rung, biển treo lệch)
        translate=0.1,
        scale=0.5,           # nhiều khoảng cách
        perspective=0.0005,  # nhìn xiên
        hsv_h=0.015, hsv_s=0.6, hsv_v=0.5,   # ánh sáng xưởng thay đổi
        mosaic=1.0,
        close_mosaic=15,
    )
    metrics = model.val()
    print(f"mAP50 = {metrics.box.map50:.3f}   mAP50-95 = {metrics.box.map:.3f}")
    best = Path(model.trainer.best)
    print(f"Trọng số tốt nhất: {best}")
    if args.export:
        sys.path.insert(0, str(PROJECT_DIR))
        from training.export_model import export
        export(best, args.imgsz)
    else:
        print("Bàn giao cho người viết code: best.pt + args.yaml + results.csv (cùng thư mục run)")


if __name__ == "__main__":
    main()
