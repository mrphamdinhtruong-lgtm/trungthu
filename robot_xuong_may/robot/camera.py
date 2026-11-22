# -*- coding: utf-8 -*-
"""
Đọc camera trong luồng riêng, luôn giữ khung hình MỚI NHẤT (không dồn trễ).
  * picamera2 : Raspberry Pi Camera Module 2 (cổng CSI) trên Raspberry Pi OS Bookworm
  * opencv    : webcam USB, hoặc file video để thử (--video)
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Tuple

log = logging.getLogger("camera")


class Camera:
    def __init__(self, backend: str = "auto", index: int = 0, width: int = 640, height: int = 480,
                 flip: str = "", video_path: Optional[str] = None):
        self.backend = backend
        self.index = index
        self.width = width
        self.height = height
        self.flip = flip or ""
        self.video_path = video_path
        self._frame = None
        self._t = 0.0
        self._count = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._picam = None
        self._cap = None
        self.active_backend = ""

    # ----------------------------------------------------------------
    def start(self) -> "Camera":
        if self.video_path:
            self._open_opencv(self.video_path)
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
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()
        return self

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

    # ----------------------------------------------------------------
    def _loop(self) -> None:
        import cv2
        is_video = self.active_backend == "video"
        fps = 0.0
        if is_video:
            fps = self._cap.get(cv2.CAP_PROP_FPS) or 25.0
        while not self._stop.is_set():
            try:
                if self._picam is not None:
                    frame = self._picam.capture_array("main")
                    if frame.ndim == 3 and frame.shape[2] == 4:
                        frame = frame[:, :, :3]
                else:
                    ok, frame = self._cap.read()
                    if not ok:
                        if is_video:              # hết video -> phát lại từ đầu
                            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                            continue
                        time.sleep(0.05)
                        continue
                    if frame.shape[1] != self.width or frame.shape[0] != self.height:
                        frame = cv2.resize(frame, (self.width, self.height))
                frame = self._apply_flip(frame)
                with self._lock:
                    self._frame = frame
                    self._t = time.monotonic()
                    self._count += 1
                if is_video:
                    time.sleep(1.0 / fps)
            except Exception as exc:
                log.error("Lỗi đọc camera: %s", exc)
                time.sleep(0.2)

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
            self._thread.join(timeout=1.0)
        try:
            if self._picam is not None:
                self._picam.stop()
                self._picam.close()
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
