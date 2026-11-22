# -*- coding: utf-8 -*-
"""Kiểu dữ liệu & tiện ích dùng chung."""
from __future__ import annotations

import math
import unicodedata
from dataclasses import dataclass


@dataclass
class Detection:
    """Một đối tượng YOLO phát hiện được. Tọa độ chuẩn hóa 0..1 theo kích thước ảnh."""
    cls: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def w(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def h(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.w * self.h


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def wrap180(deg: float) -> float:
    """Đưa góc về khoảng [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def strip_accents(text: str) -> str:
    """Bỏ dấu tiếng Việt (cho LCD 16x2 / console không hỗ trợ Unicode)."""
    text = text.replace("Đ", "D").replace("đ", "d")
    nfkd = unicodedata.normalize("NFKD", text)
    return "".join(c for c in nfkd if not unicodedata.combining(c))


def focal_norm(vfov_deg: float) -> float:
    """Tiêu cự chuẩn hóa theo chiều cao ảnh: f/H = 1 / (2 tan(VFOV/2))."""
    return 1.0 / (2.0 * math.tan(math.radians(vfov_deg) / 2.0))
