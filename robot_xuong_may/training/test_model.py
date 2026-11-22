#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
THỬ MÔ HÌNH: chạy YOLO trên camera / ảnh / video, hiển thị khung, đo FPS trên Pi.

  python training/test_model.py                         # camera của robot
  python training/test_model.py --source dataset/images/val
  python training/test_model.py --source video.mp4 --model models/best.pt
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
from robot.common import focal_norm  # noqa: E402
from robot.detector import YoloDetector  # noqa: E402


def draw(frame, dets, fps, ms):
    import cv2
    h, w = frame.shape[:2]
    fn = focal_norm(cfg.CAMERA_VFOV_DEG)
    for d in dets:
        x1, y1, x2, y2 = int(d.x1 * w), int(d.y1 * h), int(d.x2 * w), int(d.y2 * h)
        col = (0, 0, 255) if d.cls in cfg.OBSTACLE_CLASSES else (0, 220, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        txt = f"{d.cls} {d.conf:.2f}"
        station = next((s for s in cfg.STATIONS.values() if s["class"] == d.cls), None)
        if station:
            txt += f" ~{fn * station['marker_h_m'] / max(d.h, 1e-3):.2f}m"
        cv2.putText(frame, txt, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    cv2.putText(frame, f"{fps:.1f} FPS  {ms:.0f} ms", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=cfg.MODEL_PATH)
    ap.add_argument("--source", default="camera", help="camera | thư mục ảnh | file video")
    ap.add_argument("--imgsz", type=int, default=cfg.YOLO_IMGSZ)
    ap.add_argument("--conf", type=float, default=cfg.YOLO_CONF)
    args = ap.parse_args()

    import cv2
    det = YoloDetector(args.model, args.imgsz, args.conf, cfg.YOLO_IOU)
    gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or os.name == "nt")
    src = Path(args.source)
    if args.source != "camera" and src.is_dir():
        files = sorted(p for p in src.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        for p in files:
            frame = cv2.imread(str(p))
            dets = det.detect(frame)
            print(p.name, [(d.cls, round(d.conf, 2)) for d in dets])
            if gui:
                cv2.imshow("test", draw(frame, dets, 0, det.last_ms))
                if cv2.waitKey(0) & 0xFF == 27:
                    break
        return
    from robot.camera import Camera
    cam = Camera(cfg.CAMERA_BACKEND, cfg.CAMERA_INDEX, cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT, cfg.CAMERA_FLIP,
                 video_path=None if args.source == "camera" else args.source).start()
    fps, last = 0.0, time.monotonic()
    try:
        while True:
            frame, _ = cam.read()
            if frame is None:
                time.sleep(0.05)
                continue
            dets = det.detect(frame)
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(1e-3, now - last) if fps else 1.0 / max(1e-3, now - last)
            last = now
            if gui:
                cv2.imshow("test", draw(frame.copy(), dets, fps, det.last_ms))
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            else:
                print(f"{fps:5.1f} FPS {det.last_ms:5.0f} ms", [(d.cls, round(d.conf, 2)) for d in dets])
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
