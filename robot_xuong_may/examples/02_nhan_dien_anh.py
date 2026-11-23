#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BÀI 2 – NHẬN DIỆN KHỐI SỐ TRÊN 1 ẢNH BẰNG MÔ HÌNH YOLO (best.onnx)  (chạy trên: Pi hoặc laptop)

Học được:
  * Mô hình YOLO nhận 1 ảnh, trả về danh sách "khung" (bounding box): tên lớp, độ tin cậy, tọa độ.
  * Tọa độ trong code là CHUẨN HÓA 0..1 (x1 = 0.25 nghĩa là 25 % bề ngang ảnh) -> nhân với kích thước
    ảnh để ra điểm ảnh. Nhờ vậy code không phụ thuộc cỡ ảnh.
  * Khoảng cách tới khối ước lượng từ CHIỀU CAO khung: khối càng gần trông càng cao.
      khoảng_cách ≈ f × chiều_cao_thật / chiều_cao_khung,   f = 1 / (2·tan(góc_nhìn_dọc/2))

Chạy (cần models/best.onnx + models/best.metadata.json do bước 7 tạo ra):
  python examples/02_nhan_dien_anh.py anh_thu.jpg
  python examples/02_nhan_dien_anh.py anh_thu.jpg --model models/best.onnx --conf 0.3
Kết quả: in danh sách khung + lưu ảnh có vẽ khung: bai2_ket_qua.jpg
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from robot import config as cfg
from robot.common import focal_norm
from robot.detector import create_detector

ap = argparse.ArgumentParser()
ap.add_argument("image", help="đường dẫn ảnh .jpg/.png")
ap.add_argument("--model", default=cfg.MODEL_PATH)
ap.add_argument("--conf", type=float, default=cfg.YOLO_CONF, help="ngưỡng tin cậy (0..1)")
args = ap.parse_args()

img = cv2.imread(args.image)                       # 1) đọc ảnh -> mảng (cao, rộng, 3) BGR
if img is None:
    sys.exit(f"Không đọc được ảnh {args.image}")
H, W = img.shape[:2]

det = create_detector(args.model, cfg.YOLO_IMGSZ, args.conf, cfg.YOLO_IOU)   # 2) nạp mô hình
print("Mô hình:", det.version, "| lớp:", det.names)

ket_qua = det.detect(img)                          # 3) nhận diện
print(f"Tìm thấy {len(ket_qua)} khung (ảnh {W}x{H}):")
f = focal_norm(cfg.CAMERA_VFOV_DEG)
for d in ket_qua:
    x1, y1, x2, y2 = int(d.x1 * W), int(d.y1 * H), int(d.x2 * W), int(d.y2 * H)   # 0..1 -> điểm ảnh
    tram = next((s for s in cfg.STATIONS.values() if s["class"] == d.cls), None)
    kc = f * tram["marker_h_m"] / max(d.h, 1e-3) if tram else None
    print(f"  {d.cls:7s} tin cậy {d.conf:.2f}  khung ({x1},{y1})-({x2},{y2})"
          + (f"  ~ cách {kc:.2f} m" if kc else ""))
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 0), 2)                         # 4) vẽ khung
    cv2.putText(img, f"{d.cls} {d.conf:.2f}", (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 220, 0), 2)
cv2.imwrite("bai2_ket_qua.jpg", img)
print("Đã lưu ảnh kết quả: bai2_ket_qua.jpg")
