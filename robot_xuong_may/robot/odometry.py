# -*- coding: utf-8 -*-
"""Đo hành trình từ tốc độ 4 bánh (encoder) + góc quay con quay MPU-6050 (nếu có) – dùng để giãn cách
ảnh mốc khi dạy lộ trình và nhớ vật cản. Bánh mecanum trượt nhiều khi QUAY (nhất là trên gạch bóng) nên
hướng lấy từ con quay chính xác hơn nhiều so với tính từ bánh; việc dẫn đường chính vẫn dựa vào camera."""
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
        self._yaw: Optional[float] = None

    def update(self, wheel_rpm: Sequence[float], t: float, yaw_deg: Optional[float] = None) -> None:
        """yaw_deg: góc từ con quay (độ, dương = quay trái) – có thì dùng thay cho góc tính từ bánh."""
        if self._t is None:
            self._t = t
            self._yaw = yaw_deg
            return
        dt = min(0.2, max(0.0, t - self._t))
        self._t = t
        vx, vy, wz = self.kin.forward(wheel_rpm)
        dth = wz * dt
        if yaw_deg is not None and self._yaw is not None:
            d = math.radians((yaw_deg - self._yaw + 180.0) % 360.0 - 180.0)
            if abs(d) < math.radians(45.0):          # nhảy lớn = Arduino vừa reset / lệnh G -> bỏ qua 1 lần
                dth = d
        self._yaw = yaw_deg
        th_mid = self.th + 0.5 * dth
        c, s = math.cos(th_mid), math.sin(th_mid)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.th += dth
        self.dist += vx * dt
        self.path += math.hypot(vx, vy) * dt

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.th)
