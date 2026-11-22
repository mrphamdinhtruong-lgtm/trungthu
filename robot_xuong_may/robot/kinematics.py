# -*- coding: utf-8 -*-
"""
Động học robot 4 bánh mecanum (bố trí con lăn hình chữ X khi nhìn từ trên xuống).

        TRƯỚC
   FL \\       / FR        vx: tiến (+), vy: sang trái (+), wz: quay trái/CCW (+)
        |  ^  |
   RL /       \\ RR        Lx = 1/2 chiều dài cơ sở, Ly = 1/2 chiều rộng vết bánh
        SAU

Động học ngược (tốc độ góc bánh, rad/s):
    w_FL = (vx - vy - (Lx+Ly)*wz) / R
    w_FR = (vx + vy + (Lx+Ly)*wz) / R
    w_RL = (vx + vy - (Lx+Ly)*wz) / R
    w_RR = (vx - vy + (Lx+Ly)*wz) / R
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

RADS_TO_RPM = 60.0 / (2.0 * math.pi)


class MecanumKinematics:
    def __init__(self, wheel_radius: float, half_wheelbase: float, half_track: float,
                 rpm_limit: float):
        self.R = float(wheel_radius)
        self.k = float(half_wheelbase + half_track)
        self.rpm_limit = float(rpm_limit)

    @classmethod
    def from_config(cls, cfg) -> "MecanumKinematics":
        return cls(cfg.WHEEL_RADIUS_M, cfg.HALF_WHEELBASE_M, cfg.HALF_TRACK_M, cfg.WHEEL_RPM_LIMIT)

    def inverse(self, vx: float, vy: float, wz: float) -> List[float]:
        """(vx m/s, vy m/s, wz rad/s) -> [FL, FR, RL, RR] RPM.
        Nếu bánh nào vượt giới hạn thì co cả 4 bánh theo cùng tỉ lệ (giữ đúng hướng chuyển động)."""
        R, k = self.R, self.k
        w = [
            (vx - vy - k * wz) / R,
            (vx + vy + k * wz) / R,
            (vx + vy - k * wz) / R,
            (vx - vy + k * wz) / R,
        ]
        rpm = [x * RADS_TO_RPM for x in w]
        peak = max(abs(x) for x in rpm)
        if peak > self.rpm_limit > 0:
            s = self.rpm_limit / peak
            rpm = [x * s for x in rpm]
        return rpm

    def forward(self, rpm: Sequence[float]) -> Tuple[float, float, float]:
        """[FL, FR, RL, RR] RPM -> (vx, vy, wz) – dùng cho mô phỏng / hiển thị."""
        fl, fr, rl, rr = (x / RADS_TO_RPM for x in rpm)
        R, k = self.R, self.k
        vx = R / 4.0 * (fl + fr + rl + rr)
        vy = R / 4.0 * (-fl + fr + rl - rr)
        wz = R / (4.0 * k) * (-fl + fr - rl + rr)
        return vx, vy, wz

    def max_speed(self) -> float:
        """Tốc độ tiến thẳng tối đa (m/s) ứng với giới hạn RPM."""
        return self.rpm_limit / RADS_TO_RPM * self.R
