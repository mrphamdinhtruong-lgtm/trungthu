# -*- coding: utf-8 -*-
"""
Nhận diện trạm & vật cản bằng YOLO (Ultralytics YOLOv8 / YOLO11).

Trên Raspberry Pi 4 nên dùng mô hình đã xuất NCNN, imgsz=320:
    python training/export_model.py --weights runs/detect/train/weights/best.pt
-> thư mục models/best_ncnn_model  (đặt MODEL_PATH trong config.py)
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import List

from .common import Detection

log = logging.getLogger("yolo")
PROJECT_DIR = Path(__file__).resolve().parent.parent


def resolve_model_path(path: str) -> Path:
    p = Path(path)
    if not p.is_absolute():
        p = PROJECT_DIR / p
    return p


class YoloDetector:
    def __init__(self, model_path: str, imgsz: int = 320, conf: float = 0.45, iou: float = 0.5):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("Chưa cài ultralytics: pip install ultralytics") from exc
        p = resolve_model_path(model_path)
        if not p.exists():
            raise FileNotFoundError(
                f"Không thấy mô hình '{p}'. Huấn luyện theo training/README rồi xuất NCNN, "
                f"hoặc đặt MODEL_PATH trong robot/config.py")
        log.info("Nạp mô hình YOLO: %s", p)
        self.model = YOLO(str(p), task="detect")
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.names = dict(getattr(self.model, "names", {}) or {})
        self.last_ms = 0.0

    def warmup(self, frame) -> None:
        """Lần suy luận đầu tiên rất chậm -> chạy trước khi robot bắt đầu."""
        t0 = time.monotonic()
        for _ in range(2):
            self.detect(frame)
        log.info("YOLO sẵn sàng (khởi động %.1f s), lớp: %s", time.monotonic() - t0, self.names)

    def detect(self, frame) -> List[Detection]:
        t0 = time.monotonic()
        res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou, verbose=False)[0]
        self.last_ms = (time.monotonic() - t0) * 1000.0
        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            return []
        names = res.names or self.names
        xyxyn = boxes.xyxyn.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clss = boxes.cls.cpu().numpy().astype(int)
        out = []
        for (x1, y1, x2, y2), cf, c in zip(xyxyn, confs, clss):
            out.append(Detection(str(names.get(int(c), c)), float(cf), float(x1), float(y1), float(x2), float(y2)))
        return out


class NullDetector:
    """Dùng khi chưa có mô hình: robot vẫn chạy chế độ tay, không nhận diện."""
    last_ms = 0.0
    names = {}

    def warmup(self, frame) -> None:
        pass

    def detect(self, frame) -> List[Detection]:
        return []
