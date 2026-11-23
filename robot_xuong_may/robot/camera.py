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


def _webcam_apis():
    """Thứ tự trình điều khiển webcam theo hệ điều hành (Windows: DirectShow mở nhanh, nhận cả webcam USB rời)."""
    import sys
    import cv2
    if sys.platform.startswith("win"):
        return [getattr(cv2, "CAP_DSHOW", None), getattr(cv2, "CAP_MSMF", None), None]
    if sys.platform.startswith("linux"):
        return [getattr(cv2, "CAP_V4L2", None), None]
    return [None]


class _QuietCV:
    """Tắt tạm các dòng WARN/ERROR của OpenCV khi dò camera không tồn tại (làm người mới tưởng lỗi)."""

    def __enter__(self):
        import cv2
        self._lg = getattr(getattr(cv2, "utils", None), "logging", None)
        self._old = None
        if self._lg is not None:
            try:
                self._old = self._lg.getLogLevel()
                self._lg.setLogLevel(self._lg.LOG_LEVEL_SILENT)
            except Exception:
                self._lg = None
        return self

    def __exit__(self, *exc):
        if self._lg is not None and self._old is not None:
            try:
                self._lg.setLogLevel(self._old)
            except Exception:
                pass
        return False


def open_webcam(index: int):
    """Mở webcam số `index` (0 = camera đầu tiên). Trả về cv2.VideoCapture (có thể chưa mở được)."""
    import cv2
    cap = None
    with _QuietCV():
        for api in _webcam_apis():
            cap = cv2.VideoCapture(index) if api is None else cv2.VideoCapture(index, api)
            if cap.isOpened():
                return cap
            cap.release()
    return cap


def list_webcams(max_index: int = 5):
    """Dò các webcam đang cắm: [(số thứ tự, rộng, cao)]. Laptop: 0 thường là camera liền máy, 1 là camera USB rời."""
    found = []
    for i in range(max_index):
        cap = open_webcam(i)
        try:
            if cap is not None and cap.isOpened():
                with _QuietCV():
                    ok, frame = cap.read()
                if ok and frame is not None:
                    found.append((i, frame.shape[1], frame.shape[0]))
        finally:
            if cap is not None:
                cap.release()
    return found


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
            cap = open_webcam(source)
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
