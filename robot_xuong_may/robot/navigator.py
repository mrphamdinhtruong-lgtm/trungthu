# -*- coding: utf-8 -*-
"""
BỘ LÁI CỤC BỘ (local planner) – lách vật cản theo khe trống mà camera thấy trên sàn.

Từ "cảm biến khoảng cách ảo" (FreeSpace) tính chiều dài đường trống theo từng hướng θ cho
một làn rộng bằng thân robot (2·(ROBOT_HALF_WIDTH_M + NAV_SAFETY_MARGIN_M)):

    điểm vật cản p = (X, Y);  u = (cos θ, sin θ)
    s = p·u (khoảng cách dọc làn),  l = |p × u| (lệch khỏi tim làn)
    chặn làn nếu 0 < s < tầm nhìn và l < nửa bề rộng   ->   L(θ) = min s

  * Hướng mong muốn (về phía biển trạm / theo lộ trình đã dạy) còn trống -> giữ nguyên lệnh.
  * Bị chặn -> chọn hướng θ* có điểm = L(θ)/tầm_nhìn − w·|θ − θ_mong_muốn| lớn nhất:
      - "strafe" (mecanum): đi chéo theo θ*, vẫn giữ đầu robot / camera nhìn về đích
      - "turn": quay đầu về θ*
  * Không còn khe nào đủ rộng -> "blocked" (máy trạng thái chuyển sang GẶP VẬT CẢN).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .common import clamp


@dataclass
class PlanResult:
    vx: float
    vy: float
    wz: float
    status: str          # none | clear | avoid | blocked
    theta: Optional[float] = None
    free_len: float = float("inf")


class LocalPlanner:
    def __init__(self, cfg):
        self.cfg = cfg
        self.half_width = float(cfg.ROBOT_HALF_WIDTH_M + cfg.NAV_SAFETY_MARGIN_M)
        half_fov = math.radians(cfg.CAMERA_HFOV_DEG) / 2.0
        self.half_fov = half_fov
        self.cands = np.linspace(-0.9 * half_fov, 0.9 * half_fov, 19)
        self.prev_theta = 0.0

    def free_length(self, X: np.ndarray, Y: np.ndarray, theta: float, lookahead: float) -> float:
        if X.size == 0:
            return lookahead
        c, s = math.cos(theta), math.sin(theta)
        along = X * c + Y * s
        lat = np.abs(X * s - Y * c)
        hit = (along > 0) & (along < lookahead) & (lat < self.half_width)
        return float(along[hit].min()) if np.any(hit) else lookahead

    def lane_distance(self, free, theta: float = 0.0) -> float:
        """Khoảng cách tới vật cản gần nhất trong làn theo hướng chuyển động theta (0 = thẳng)."""
        if free is None or not free.valid:
            return float("inf")
        X, Y = free.points()
        L = self.free_length(X, Y, theta, free.max_range)
        return L if L < free.max_range else float("inf")

    def adjust(self, vx: float, vy: float, wz: float, free, desired: float,
               lookahead: float, stop_dist: float) -> PlanResult:
        """Hiệu chỉnh lệnh (vx, vy, wz) theo khe trống. desired: hướng mong muốn (rad, + trái)."""
        c = self.cfg
        if free is None or not free.valid:
            return PlanResult(vx, vy, wz, "none", desired)
        if abs(vx) < 0.01 and abs(vy) < 0.01:
            return PlanResult(vx, vy, wz, "clear", desired)      # quay tại chỗ: không tịnh tiến vào vật
        lookahead = max(0.3, lookahead)
        X, Y = free.points()
        slow = max(stop_dist + 0.1, c.NAV_SLOW_DISTANCE_M)
        L_des = self.free_length(X, Y, desired, lookahead)
        if L_des >= lookahead - 1e-6:
            ahead = self.free_length(X, Y, 0.0, lookahead)
            k = 1.0 if ahead >= lookahead - 1e-6 else clamp((ahead - stop_dist) / (slow - stop_dist), 0.3, 1.0)
            self.prev_theta = desired
            return PlanResult(vx * k, vy, wz, "clear", desired, L_des)

        # 1) ưu tiên các hướng TRỐNG HOÀN TOÀN trong tầm nhìn -> chọn hướng gần đích nhất (kiểu VFH)
        # 2) không có -> chọn hướng có khoảng trống dài nhất (còn hơn khoảng dừng)
        best, best_score, best_L = None, -1e9, 0.0
        lengths = [(float(th), self.free_length(X, Y, float(th), lookahead)) for th in self.cands]
        free_dirs = [(th, L) for th, L in lengths if L >= lookahead - 1e-6]
        for th, L in (free_dirs or lengths):
            if L < stop_dist + 0.15:
                continue
            dev = (c.NAV_W_DIR * abs(th - desired) + 0.15 * abs(th - self.prev_theta)) / self.half_fov
            score = -dev if free_dirs else (L / lookahead - dev)
            if score > best_score:
                best, best_score, best_L = th, score, L
        if best is None:
            return PlanResult(0.0, 0.0, 0.0, "blocked", None, max(L_des, 0.0))

        self.prev_theta = best
        v = max(abs(vx), 0.06) if vx > 0 else 0.0
        v *= clamp((best_L - stop_dist) / (slow - stop_dist), 0.25, 1.0)
        if c.NAV_AVOID_MODE == "strafe":
            # đi theo hướng θ*, giữ nguyên lệnh quay (camera vẫn nhìn đích)
            return PlanResult(v * math.cos(best), v * math.sin(best), wz, "avoid", best, best_L)
        w = clamp(c.NAV_K_TURN * best, -c.W_MAX, c.W_MAX)
        return PlanResult(v * max(0.3, math.cos(best)), 0.0, w, "avoid", best, best_L)


class ObstacleMemory:
    """Nhớ ngắn hạn các điểm vật cản trong hệ tọa độ hành trình (odometry).

    Camera chỉ nhìn ±31° phía trước: khi lách qua, vật cản lọt ra BÊN HÔNG robot hoặc
    xuống DƯỚI đáy khung hình nên không còn thấy. Bộ nhớ giữ các điểm đó vài giây để robot
    không quẹt vào. Điểm nằm trong vùng camera đang nhìn thấy được thay bằng quan sát mới
    (vật đã đi khỏi thì quên ngay)."""

    def __init__(self, geom, keep_s: float = 10.0, max_points: int = 2500, max_range: float = 3.0,
                 depth: tuple = (0.0, 0.15, 0.3)):
        self.geom = geom
        self.keep_s = keep_s
        self.max_points = max_points
        self.max_range = max_range
        self.near_blind = geom.ground_distance(1.0)          # gần hơn đáy khung -> không thấy
        self.half_fov = geom.hfov / 2.0
        self.depth = depth            # camera chỉ thấy MẶT TRƯỚC vật -> giả định vật dày tối thiểu 0.3 m
        self.pts = np.zeros((0, 2))
        self.t = np.zeros(0)

    def reset(self) -> None:
        self.pts = np.zeros((0, 2))
        self.t = np.zeros(0)

    @staticmethod
    def _to_world(X, Y, pose):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        return np.stack([x + X * c - Y * s, y + X * s + Y * c], axis=1)

    @staticmethod
    def _to_robot(P, pose):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        dx, dy = P[:, 0] - x, P[:, 1] - y
        return dx * c + dy * s, -dx * s + dy * c

    def _in_view(self, X, Y):
        b = np.arctan2(Y, np.maximum(X, 1e-6))
        return (X >= self.near_blind + 0.05) & (X <= self.max_range) & (np.abs(b) < 0.95 * self.half_fov)

    def update(self, X: np.ndarray, Y: np.ndarray, pose, now: float) -> None:
        if self.pts.size:
            Xr, Yr = self._to_robot(self.pts, pose)
            keep = (now - self.t <= self.keep_s) & ~self._in_view(Xr, Yr) & (Xr > -0.4) & (np.hypot(Xr, Yr) < 2.0)
            self.pts, self.t = self.pts[keep], self.t[keep]
        if X.size:
            r = np.hypot(X, Y)
            ux, uy = X / np.maximum(r, 1e-6), Y / np.maximum(r, 1e-6)
            Xd = np.concatenate([X + d * ux for d in self.depth])
            Yd = np.concatenate([Y + d * uy for d in self.depth])
            self.pts = np.concatenate([self.pts, self._to_world(Xd, Yd, pose)])
            self.t = np.concatenate([self.t, np.full(Xd.size, now)])
        if len(self.t) > self.max_points:
            self.pts, self.t = self.pts[-self.max_points:], self.t[-self.max_points:]

    def recall(self, pose):
        """Các điểm đã nhớ đang NGOÀI tầm nhìn, đổi về hệ tọa độ robot hiện tại."""
        if not self.pts.size:
            return np.zeros(0), np.zeros(0)
        Xr, Yr = self._to_robot(self.pts, pose)
        m = ~self._in_view(Xr, Yr) & (Xr > -0.4)
        return Xr[m], Yr[m]
