# -*- coding: utf-8 -*-
"""
Hình học camera một mắt nhìn sàn phẳng (pinhole):
  * hàng ảnh  -> khoảng cách TIẾN tới điểm trên sàn (m)
  * cột ảnh   -> lệch NGANG (m) tại khoảng cách đó
  * điểm sàn  -> tọa độ ảnh (để vẽ làn đường của robot lên màn hình)

Tọa độ ảnh chuẩn hóa: x, y trong [0, 1] (x sang phải, y xuống dưới).
Tọa độ robot: X tiến về trước, Y sang TRÁI (giống hệ trục vx, vy).
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from .common import focal_norm


class CameraGeometry:
    def __init__(self, hfov_deg: float, vfov_deg: float, height_m: float, tilt_deg: float = 0.0):
        self.fx = focal_norm(hfov_deg)       # tiêu cự chuẩn hóa theo bề ngang
        self.fy = focal_norm(vfov_deg)       # tiêu cự chuẩn hóa theo chiều cao
        self.h = float(height_m)
        self.tilt = math.radians(tilt_deg)
        self.hfov = math.radians(hfov_deg)

    @classmethod
    def from_config(cls, cfg) -> "CameraGeometry":
        return cls(cfg.CAMERA_HFOV_DEG, cfg.CAMERA_VFOV_DEG, cfg.CAMERA_HEIGHT_M, cfg.CAMERA_TILT_DEG)

    def ground_distance(self, y: float) -> float:
        """Khoảng cách tiến (m) tới điểm sàn ở hàng y; inf nếu ở trên đường chân trời."""
        ang = self.tilt + math.atan((y - 0.5) / self.fy)
        if ang <= 1e-3:
            return float("inf")
        return self.h / math.tan(ang)

    def row_of_distance(self, d: float) -> float:
        """Hàng ảnh y (0..1) của điểm sàn cách d mét (có thể > 1 nếu nằm dưới khung)."""
        ang = math.atan2(self.h, d) - self.tilt
        return 0.5 + self.fy * math.tan(ang)

    def _cos_factor(self, y: float) -> float:
        v = (y - 0.5) / self.fy
        return math.cos(self.tilt) - v * math.sin(self.tilt)

    def lateral(self, x: float, forward: float, y: Optional[float] = None) -> float:
        """Lệch ngang Y (m, dương = trái) của điểm sàn ở cột x cách `forward` mét (tính cả góc chúc)."""
        if y is None:
            y = self.row_of_distance(forward)
        return forward * (0.5 - x) / self.fx / self._cos_factor(y)

    def bearing(self, x: float) -> float:
        """Góc phương vị (rad, dương = trái) của cột x."""
        return math.atan((0.5 - x) / self.fx)

    def column_of_bearing(self, b: float) -> float:
        return 0.5 - math.tan(b) * self.fx

    def image_of_ground(self, X: float, Y: float) -> Optional[Tuple[float, float]]:
        """Điểm sàn (X tiến, Y trái) -> (x, y) ảnh chuẩn hóa."""
        if X <= 1e-3:
            return None
        y = self.row_of_distance(X)
        return 0.5 - Y / X * self.fx * self._cos_factor(y), y
