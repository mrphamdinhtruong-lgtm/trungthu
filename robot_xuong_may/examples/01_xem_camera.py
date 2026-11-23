#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BÀI 1 – MỞ CAMERA VÀ XEM ẢNH  (chạy trên: Raspberry Pi hoặc laptop)

Học được:
  * Một "khung hình" (frame) trong OpenCV là mảng số numpy có kích thước (cao, rộng, 3)
    – ở đây (480, 640, 3): 480 hàng điểm ảnh, 640 cột, mỗi điểm 3 số màu theo thứ tự B, G, R (0..255).
  * Đọc camera trong luồng riêng (robot/camera.py) để lúc nào cũng có ảnh MỚI NHẤT.
  * FPS = số khung hình xử lý được mỗi giây.

Chạy:
  python examples/01_xem_camera.py                  # camera robot (Pi) hoặc webcam (laptop)
  python examples/01_xem_camera.py --video thu.mp4  # file video
  python examples/01_xem_camera.py --frames 30      # không cần màn hình: đọc 30 khung rồi dừng
  python examples/01_xem_camera.py --list           # liệt kê các webcam đang cắm (0, 1, 2...)
  python examples/01_xem_camera.py --camera 1       # dùng webcam số 1 (camera USB cắm ngoài laptop)
Nhấn ESC để thoát cửa sổ.
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # để import được thư mục robot/

import cv2

from robot import config as cfg
from robot.camera import Camera

ap = argparse.ArgumentParser()
ap.add_argument("--video", default=None, help="file video hoặc thư mục ảnh thay cho camera")
ap.add_argument("--frames", type=int, default=0, help="đọc N khung rồi dừng (0 = chạy mãi)")
ap.add_argument("--camera", type=int, default=None, help="số webcam (0 = liền máy, 1 = USB rời...) – thay CAMERA_INDEX")
ap.add_argument("--list", action="store_true", help="chỉ liệt kê các webcam đang cắm rồi thoát")
args = ap.parse_args()

if args.list:
    from robot.camera import list_webcams
    cams = list_webcams()
    for i, w, h in cams:
        print(f"  webcam {i}: có hình {w}x{h}   -> dùng: --camera {i}")
    if not cams:
        print("Không thấy webcam nào (kiểm tra cáp USB / camera đang bị chương trình khác dùng)")
    sys.exit(0)

# 1) Mở camera theo cấu hình trong robot/config.py (picamera2 trên Pi, webcam trên laptop)
backend = "opencv" if args.camera is not None else cfg.CAMERA_BACKEND      # chọn số webcam -> bỏ qua picamera2
index = cfg.CAMERA_INDEX if args.camera is None else args.camera
try:
    cam = Camera(backend, index, cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT, cfg.CAMERA_FLIP, video_path=args.video).start()
except RuntimeError as exc:
    raise SystemExit(f"{exc}\n  -> Xem các webcam đang có: python examples/01_xem_camera.py --list  (rồi dùng --camera <số>)")
print("Nguồn hình:", cam.active_backend)
co_man_hinh = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or os.name == "nt")

dem, t0 = 0, time.monotonic()
try:
    while True:
        frame, luc_chup = cam.read()           # 2) lấy khung MỚI NHẤT
        if frame is None:                      #    camera chưa kịp có ảnh
            time.sleep(0.05)
            continue
        dem += 1
        h, w = frame.shape[:2]
        b, g, r = frame[h // 2, w // 2]        # 3) màu điểm ảnh chính giữa (B, G, R)
        fps = dem / (time.monotonic() - t0)
        if dem % 10 == 1:
            print(f"khung {dem}: kích thước {frame.shape}, điểm giữa B={b} G={g} R={r}, {fps:.1f} FPS")
        if co_man_hinh:
            cv2.putText(frame, f"{fps:.1f} FPS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("Bai 1 - camera", frame)
            if cv2.waitKey(1) & 0xFF == 27:    # ESC
                break
        if args.frames and dem >= args.frames:
            cv2.imwrite("bai1_khung_cuoi.jpg", frame)
            print("Đã lưu khung cuối: bai1_khung_cuoi.jpg")
            break
        time.sleep(0.03)
finally:
    cam.stop()
    cv2.destroyAllWindows()
