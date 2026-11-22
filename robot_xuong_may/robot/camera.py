# -*- coding: utf-8 -*-
"""
Đọc hình trong luồng riêng, luôn giữ khung hình MỚI NHẤT (không dồn trễ).

Nguồn hình:
  * picamera2 : Raspberry Pi Camera Module 2 (cổng CSI) trên Raspberry Pi OS Bookworm
  * opencv    : webcam USB (chỉ số 0, 1...)
  * file video: .mp4 / .avi ... (phát lặp lại)          -> thử bằng video nhóm train gửi
  * thư mục ảnh: .jpg / .png (trình chiếu image_fps ảnh/giây, lặp lại) -> thử bằng ảnh test

XỬ LÝ MẤT HÌNH:
  * Khung hình không mới trong CAMERA_TIMEOUT_S -> main.py báo "Lỗi camera", robot DỪNG.
  * Luồng camera tự mở lại thiết bị sau RECONNECT_AFTER lần đọc lỗi liên tiếp (rút / cắm lại cáp USB,
    lỗi libcamera), có giãn cách để không làm treo CPU. Hình về lại -> hết lỗi -> nhấn START chạy tiếp.
"""
from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

log = logging.getLogger("camera")
IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp"}
RECONNECT_AFTER = 20          # số lần đọc lỗi liên tiếp trước khi mở lại camera
RECONNECT_WAIT_S = 1.0


class Camera:
    def __init__(self, backend: str = "auto", index: int = 0, width: int = 640, height: int = 480,
                 flip: str = "", video_path: Optional[str] = None, image_fps: float = 2.0):
        self.backend = backend
        self.index = index
        self.width = width
        self.height = height
        self.flip = flip or ""
        self.video_path = video_path
        self.image_fps = max(0.1, float(image_fps))
        self._frame = None
        self._t = 0.0
        self._count = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._picam = None
        self._cap = None
        self._images: List[Path] = []
        self._img_i = 0
        self.active_backend = ""
        self.source_name = ""
        self.reconnects = 0

    # ----------------------------------------------------------------
    def start(self) -> "Camera":
        self._open()
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()
        return self

    def _open(self) -> None:
        src = self.video_path
        if src and Path(src).is_dir():
            self._open_images(Path(src))
        elif src:
            self._open_opencv(src)
        elif self.backend in ("auto", "picamera2"):
            try:
                self._open_picamera2()
            except Exception as exc:
                if self.backend == "picamera2":
                    raise RuntimeError(f"Không mở được Camera Module (picamera2): {exc}") from exc
                log.info("Không dùng được picamera2 (%s) -> thử OpenCV", exc)
                self._open_opencv(self.index)
        else:
            self._open_opencv(self.index)

    def _open_images(self, folder: Path) -> None:
        self._images = sorted(p for p in folder.rglob("*") if p.suffix.lower() in IMG_EXT)
        if not self._images:
            raise RuntimeError(f"Thư mục {folder} không có ảnh")
        self.active_backend = "images"
        log.info("Nguồn ảnh: %s (%d ảnh, %.1f ảnh/s)", folder, len(self._images), self.image_fps)

    def _open_picamera2(self) -> None:
        from picamera2 import Picamera2  # cài sẵn trên Raspberry Pi OS
        cam = Picamera2()
        # "RGB888" của picamera2 cho mảng thứ tự B,G,R -> dùng thẳng với OpenCV
        conf = cam.create_video_configuration(main={"size": (self.width, self.height), "format": "RGB888"},
                                              buffer_count=4)
        cam.configure(conf)
        cam.start()
        time.sleep(0.5)
        self._picam = cam
        self.active_backend = "picamera2"
        log.info("Camera Module (picamera2) %dx%d", self.width, self.height)

    def _open_opencv(self, source) -> None:
        import cv2
        if isinstance(source, int):
            cap = cv2.VideoCapture(source, cv2.CAP_V4L2) if hasattr(cv2, "CAP_V4L2") else cv2.VideoCapture(source)
            if not cap.isOpened():
                cap = cv2.VideoCapture(source)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            cap = cv2.VideoCapture(str(source))
        if not cap.isOpened():
            raise RuntimeError(f"Không mở được camera/video: {source}")
        self._cap = cap
        self.active_backend = "opencv" if isinstance(source, int) else "video"
        log.info("Camera OpenCV: %s", source)

    def _close_device(self) -> None:
        try:
            if self._picam is not None:
                self._picam.stop()
                self._picam.close()
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        self._picam = None
        self._cap = None

    def _reconnect(self) -> None:
        """Mở lại camera sau khi đọc lỗi liên tục (cáp lỏng, rút USB, lỗi driver)."""
        self.reconnects += 1
        log.warning("Mất hình -> mở lại camera (lần %d)", self.reconnects)
        self._close_device()
        time.sleep(RECONNECT_WAIT_S)
        try:
            self._open()
        except Exception as exc:
            log.error("Mở lại camera thất bại: %s", exc)

    # ----------------------------------------------------------------
    def _read_one(self):
        """Đọc 1 khung -> ảnh BGR hoặc None nếu lỗi."""
        import cv2
        if self.active_backend == "images":
            p = self._images[self._img_i % len(self._images)]
            self._img_i += 1
            self.source_name = p.name
            frame = cv2.imread(str(p))
            time.sleep(1.0 / self.image_fps)
        elif self._picam is not None:
            frame = self._picam.capture_array("main")
            if frame is not None and frame.ndim == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]
        elif self._cap is not None:
            ok, frame = self._cap.read()
            if not ok:
                if self.active_backend == "video":          # hết video -> phát lại từ đầu
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = self._cap.read()
                if not ok:
                    return None
            if self.active_backend == "video":
                fps = self._cap.get(cv2.CAP_PROP_FPS) or 25.0
                time.sleep(1.0 / min(60.0, max(1.0, fps)))
        else:
            return None
        if frame is None:
            return None
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        return self._apply_flip(frame)

    def _loop(self) -> None:
        fails = 0
        while not self._stop.is_set():
            try:
                frame = self._read_one()
            except Exception as exc:
                log.error("Lỗi đọc camera: %s", exc)
                frame = None
            if frame is None:
                fails += 1
                if fails >= RECONNECT_AFTER and self.active_backend not in ("images",):
                    self._reconnect()
                    fails = 0
                else:
                    time.sleep(0.05)
                continue
            fails = 0
            with self._lock:
                self._frame = frame
                self._t = time.monotonic()
                self._count += 1

    def _apply_flip(self, frame):
        if not self.flip:
            return frame
        if self.flip == "hv":
            return frame[::-1, ::-1].copy()
        if self.flip == "h":
            return frame[:, ::-1].copy()
        if self.flip == "v":
            return frame[::-1].copy()
        return frame

    def read(self) -> Tuple[Optional[object], float]:
        """-> (khung hình BGR mới nhất hoặc None, thời điểm chụp time.monotonic())"""
        with self._lock:
            return self._frame, self._t

    @property
    def frame_count(self) -> int:
        return self._count

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)
        self._close_device()
