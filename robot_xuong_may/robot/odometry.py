# -*- coding: utf-8 -*-
"""Đo hành trình từ tốc độ 4 bánh (encoder) – dùng để giãn cách ảnh mốc khi dạy lộ trình.
Bánh mecanum trượt nhiều (nhất là trên gạch bóng) nên chỉ tin được ngắn hạn;
việc dẫn đường chính dựa vào camera."""
from __future__ import annotations

import math
from typing import Optional, Sequence


class Odometry:
    def __init__(self, kinematics):
        self.kin = kinematics
        self.reset()

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.dist = 0.0          # quãng đường tiến (có dấu, m)
        self.path = 0.0          # tổng quãng đường (m)
        self._t: Optional[float] = None

    def update(self, wheel_rpm: Sequence[float], t: float) -> None:
        if self._t is None:
            self._t = t
            return
        dt = min(0.2, max(0.0, t - self._t))
        self._t = t
        vx, vy, wz = self.kin.forward(wheel_rpm)
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.th += wz * dt
        self.dist += vx * dt
        self.path += math.hypot(vx, vy) * dt

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.th)
