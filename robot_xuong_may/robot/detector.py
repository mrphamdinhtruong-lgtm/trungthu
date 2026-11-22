# -*- coding: utf-8 -*-
"""
Nhận diện biển TRẠM (và vật cản nếu mô hình có lớp đó) bằng YOLO (Ultralytics YOLO11 / YOLOv8).

Hai cách chạy mô hình:
  * models/best.onnx  (+ models/best.metadata.json)  -> OnnxDetector: chỉ cần onnxruntime
    (hoặc OpenCV DNN), KHÔNG cần cài torch/ultralytics trên Raspberry Pi -> nhẹ, khởi động nhanh.
    Xuất bằng:  python training/export_model.py --weights best.pt
  * best.pt / *_ncnn_model / ...                    -> YoloDetector (thư viện ultralytics)

Kết quả luôn là danh sách Detection với tọa độ CHUẨN HÓA 0..1 theo ảnh gốc.
"""
from __future__ import annotations

import ast
import json
import logging
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .common import Detection

log = logging.getLogger("yolo")
PROJECT_DIR = Path(__file__).resolve().parent.parent


def resolve_model_path(path: str) -> Path:
    p = Path(path)
    if not p.is_absolute():
        p = PROJECT_DIR / p
    return p


def metadata_path(model_path: Path) -> Path:
    """models/best.onnx -> models/best.metadata.json"""
    return model_path.with_name(model_path.stem + ".metadata.json")


def _parse_names(names) -> Dict[int, str]:
    if isinstance(names, str):
        try:
            names = ast.literal_eval(names)
        except (ValueError, SyntaxError):
            names = json.loads(names)
    if isinstance(names, (list, tuple)):
        return {i: str(n) for i, n in enumerate(names)}
    return {int(k): str(v) for k, v in dict(names).items()}


def load_metadata(model_path: Path) -> dict:
    """Đọc best.metadata.json (ưu tiên). Không có thì đọc metadata Ultralytics nhúng trong file ONNX."""
    meta: dict = {}
    mp = metadata_path(model_path)
    if mp.exists():
        meta = json.loads(mp.read_text(encoding="utf-8"))
    elif model_path.suffix.lower() == ".onnx":
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
            meta = dict(sess.get_modelmeta().custom_metadata_map)
        except Exception as exc:                    # pragma: no cover - chỉ để chẩn đoán
            log.warning("Không đọc được metadata trong ONNX: %s", exc)
    if "names" in meta:
        meta["names"] = _parse_names(meta["names"])
    if "imgsz" in meta and isinstance(meta["imgsz"], str):
        meta["imgsz"] = ast.literal_eval(meta["imgsz"])
    return meta


def check_classes(names: Dict[int, str], cfg) -> Tuple[List[str], List[str]]:
    """So tên lớp của mô hình với config. -> (lỗi nghiêm trọng, cảnh báo)."""
    errors, warnings = [], []
    have = set(names.values())
    for sid, st in cfg.STATIONS.items():
        if st["class"] not in have:
            errors.append(f"Mô hình KHÔNG có lớp '{st['class']}' của {st.get('name', sid)}")
    expected = list(getattr(cfg, "MODEL_CLASSES", ()) or ())
    if expected:
        got = [names[i] for i in sorted(names)]
        if got != expected:
            errors.append(f"Thứ tự lớp sai: mô hình {got} ≠ config MODEL_CLASSES {expected}")
    obst = [c for c in cfg.OBSTACLE_CLASSES if c in have]
    if not obst:
        warnings.append("Mô hình không có lớp vật cản -> chỉ phát hiện vật cản bằng nhận diện sàn")
    return errors, warnings


# ---------------------------------------------------------------------------
# Tiền xử lý / hậu xử lý YOLO (giống Ultralytics)
# ---------------------------------------------------------------------------
def letterbox(img, new_shape: Tuple[int, int], color=(114, 114, 114)):
    """Co ảnh giữ tỉ lệ + đệm viền -> (ảnh, tỉ lệ, (đệm_x, đệm_y))."""
    import cv2
    h, w = img.shape[:2]
    nh, nw = new_shape
    r = min(nh / h, nw / w)
    uh, uw = int(round(h * r)), int(round(w * r))
    dw, dh = (nw - uw) / 2.0, (nh - uh) / 2.0
    if (uw, uh) != (w, h):
        img = cv2.resize(img, (uw, uh), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, r, (left, top)


def decode_yolo(out: np.ndarray, nc: int, conf: float, iou: float, ratio: float, pad: Tuple[float, float],
                orig_wh: Tuple[int, int], max_det: int = 50, names: Optional[Dict[int, str]] = None
                ) -> List[Detection]:
    """Giải mã đầu ra YOLOv8/YOLO11 dạng (1, 4+nc, N) hoặc (1, N, 4+nc): cx, cy, w, h, điểm từng lớp."""
    import cv2
    a = np.asarray(out, dtype=np.float32)
    if a.ndim == 3:
        a = a[0]
    if a.shape[0] == 4 + nc and a.shape[1] != 4 + nc:
        a = a.T                                       # -> (N, 4+nc)
    elif a.shape[1] != 4 + nc:
        raise ValueError(f"Đầu ra mô hình {np.shape(out)} không khớp số lớp nc={nc}")
    scores = a[:, 4:]
    cls = scores.argmax(axis=1)
    cf = scores[np.arange(len(a)), cls]
    keep = cf >= conf
    if not np.any(keep):
        return []
    a, cls, cf = a[keep], cls[keep], cf[keep]
    cx, cy, w, h = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    x1 = (cx - w / 2 - pad[0]) / ratio
    y1 = (cy - h / 2 - pad[1]) / ratio
    x2 = (cx + w / 2 - pad[0]) / ratio
    y2 = (cy + h / 2 - pad[1]) / ratio
    W, H = orig_wh
    x1, x2 = np.clip(x1, 0, W), np.clip(x2, 0, W)
    y1, y2 = np.clip(y1, 0, H), np.clip(y2, 0, H)
    # NMS theo từng lớp (dịch hộp theo mã lớp như Ultralytics)
    off = cls.astype(np.float32) * 7680.0
    rects = np.stack([x1 + off, y1, x2 - x1, y2 - y1], axis=1).tolist()
    idx = cv2.dnn.NMSBoxes(rects, cf.tolist(), conf, iou)
    idx = np.array(idx).reshape(-1)[:max_det] if len(idx) else []
    names = names or {}
    return [Detection(names.get(int(cls[i]), str(int(cls[i]))), float(cf[i]),
                      float(x1[i] / W), float(y1[i] / H), float(x2[i] / W), float(y2[i] / H))
            for i in idx]


class OnnxDetector:
    """Chạy best.onnx bằng onnxruntime (mặc định) hoặc OpenCV DNN (dự phòng)."""

    def __init__(self, model_path: str, imgsz: int = 416, conf: float = 0.45, iou: float = 0.5,
                 threads: int = 4, backend: str = "auto"):
        p = resolve_model_path(model_path)
        if not p.exists():
            raise FileNotFoundError(f"Không thấy mô hình '{p}' (xuất bằng training/export_model.py)")
        self.path = p
        self.meta = load_metadata(p)
        self.names: Dict[int, str] = self.meta.get("names", {})
        if not self.names:
            raise RuntimeError(f"Thiếu tên lớp: cần file {metadata_path(p).name} cạnh mô hình")
        self.nc = len(self.names)
        isz = self.meta.get("imgsz", imgsz)
        self.shape = (int(isz[0]), int(isz[1])) if isinstance(isz, (list, tuple)) else (int(isz), int(isz))
        if imgsz and isinstance(imgsz, int) and imgsz != self.shape[0]:
            log.warning("YOLO_IMGSZ=%d khác kích thước lúc xuất ONNX %s -> dùng %s", imgsz, self.shape, self.shape)
        self.conf, self.iou = conf, iou
        self.version = str(self.meta.get("model_version", "?"))
        self.last_ms = 0.0
        self._sess = None
        self._net = None
        if backend in ("auto", "onnxruntime"):
            try:
                import onnxruntime as ort
                so = ort.SessionOptions()
                so.intra_op_num_threads = threads
                self._sess = ort.InferenceSession(str(p), sess_options=so, providers=["CPUExecutionProvider"])
                self._inp = self._sess.get_inputs()[0].name
                self.backend = "onnxruntime"
            except ImportError:
                if backend == "onnxruntime":
                    raise
        if self._sess is None:
            import cv2
            self._net = cv2.dnn.readNetFromONNX(str(p))
            self.backend = "opencv-dnn"
        log.info("Nạp %s (%s): lớp %s, ảnh %s, phiên bản %s", p.name, self.backend, self.names,
                 self.shape, self.version)

    def warmup(self, frame) -> None:
        t0 = time.monotonic()
        for _ in range(2):
            self.detect(frame)
        log.info("YOLO ONNX sẵn sàng (khởi động %.1f s, %.0f ms/khung)", time.monotonic() - t0, self.last_ms)

    def detect(self, frame) -> List[Detection]:
        import cv2
        t0 = time.monotonic()
        h, w = frame.shape[:2]
        img, r, pad = letterbox(frame, self.shape)
        blob = cv2.dnn.blobFromImage(img, 1.0 / 255.0, swapRB=True)      # NCHW, RGB, 0..1
        if self._sess is not None:
            out = self._sess.run(None, {self._inp: blob})[0]
        else:
            self._net.setInput(blob)
            out = self._net.forward()
        dets = decode_yolo(out, self.nc, self.conf, self.iou, r, pad, (w, h), names=self.names)
        self.last_ms = (time.monotonic() - t0) * 1000.0
        return dets


class YoloDetector:
    """Chạy .pt / *_ncnn_model / ... bằng thư viện ultralytics."""

    def __init__(self, model_path: str, imgsz: int = 416, conf: float = 0.45, iou: float = 0.5):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("Chưa cài ultralytics (pip install ultralytics) – hoặc dùng models/best.onnx") from exc
        p = resolve_model_path(model_path)
        if not p.exists():
            raise FileNotFoundError(f"Không thấy mô hình '{p}'. Đặt MODEL_PATH trong robot/config.py")
        log.info("Nạp mô hình YOLO: %s", p)
        self.path = p
        self.model = YOLO(str(p), task="detect")
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.names = dict(getattr(self.model, "names", {}) or {})
        self.meta = load_metadata(p) if metadata_path(p).exists() else {}
        self.version = str(self.meta.get("model_version", p.name))
        self.backend = "ultralytics"
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


def create_detector(model_path: str, imgsz: int = 416, conf: float = 0.45, iou: float = 0.5):
    """.onnx -> OnnxDetector (nhẹ), còn lại -> YoloDetector (ultralytics)."""
    if str(model_path).lower().endswith(".onnx"):
        return OnnxDetector(model_path, imgsz, conf, iou)
    return YoloDetector(model_path, imgsz, conf, iou)


class NullDetector:
    """Dùng khi chưa có mô hình: robot vẫn chạy chế độ tay, không nhận diện."""
    last_ms = 0.0
    names: Dict[int, str] = {}
    version = "không có mô hình"
    backend = "none"

    def warmup(self, frame) -> None:
        pass

    def detect(self, frame) -> List[Detection]:
        return []


def detections_to_rows(dets: Sequence[Detection]) -> List[dict]:
    return [{"cls": d.cls, "conf": round(d.conf, 3), "x1": round(d.x1, 4), "y1": round(d.y1, 4),
             "x2": round(d.x2, 4), "y2": round(d.y2, 4)} for d in dets]
