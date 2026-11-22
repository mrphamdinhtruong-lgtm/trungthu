#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CHỤP ẢNH LÀM DỮ LIỆU HUẤN LUYỆN (chạy trên Raspberry Pi, đúng camera của robot)

  python training/capture_images.py                    # SPACE: chụp 1 ảnh, A: bật/tắt tự chụp, ESC: thoát
  python training/capture_images.py --interval 1.0     # tự chụp mỗi 1 s (chạy không màn hình được)
  python training/capture_images.py --prefix tram1     # đặt tiền tố tên file

Ảnh lưu ở dataset/raw/. Gợi ý: mỗi lớp 300-500 ảnh, nhiều khoảng cách (0.4-5 m), nhiều góc,
nhiều thời điểm ánh sáng, có cả ảnh KHÔNG có biển báo (ảnh nền) ~10%.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot.camera import Camera  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(PROJECT_DIR / "dataset" / "raw"))
    ap.add_argument("--interval", type=float, default=0.0, help="tự chụp mỗi N giây (0 = chụp tay)")
    ap.add_argument("--prefix", default="img")
    ap.add_argument("--count", type=int, default=0, help="dừng sau N ảnh (0 = không giới hạn)")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cam = Camera(cfg.CAMERA_BACKEND, cfg.CAMERA_INDEX, cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT, cfg.CAMERA_FLIP).start()
    import cv2
    gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or os.name == "nt")
    auto = args.interval > 0
    last = 0.0
    n = 0
    print(f"Lưu ảnh vào {out}. {'Tự chụp mỗi %.1f s' % args.interval if auto else 'SPACE để chụp'}")
    try:
        while True:
            frame, _ = cam.read()
            if frame is None:
                time.sleep(0.05)
                continue
            now = time.monotonic()
            save = auto and now - last >= max(0.2, args.interval)
            if gui:
                view = frame.copy()
                cv2.putText(view, f"{n} anh | {'AUTO' if auto else 'SPACE: chup'} | A: auto | ESC", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("capture", view)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
                if k == 32:
                    save = True
                if k in (ord("a"), ord("A")):
                    auto = not auto
                    args.interval = args.interval or 1.0
            else:
                time.sleep(0.02)
            if save:
                name = out / f"{args.prefix}_{time.strftime('%Y%m%d_%H%M%S')}_{n:04d}.jpg"
                cv2.imwrite(str(name), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
                n += 1
                last = now
                print(f"  {name.name}")
                if args.count and n >= args.count:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        if gui:
            cv2.destroyAllWindows()
    print(f"Đã chụp {n} ảnh.")


if __name__ == "__main__":
    main()
