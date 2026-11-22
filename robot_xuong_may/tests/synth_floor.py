# -*- coding: utf-8 -*-
"""
Dựng ảnh camera TỔNG HỢP của sàn xưởng (chiếu phối cảnh đúng hình học camera) để kiểm thử
bộ nhận diện sàn trống:  gạch đá hoa (ron, vân, vệt phản sáng), gạch bông nhiều màu,
bê tông (vân, vết bẩn, vết nứt, vạch sơn vàng phẳng) + vật cản dạng hộp.
"""
from __future__ import annotations

import math

import numpy as np

from robot.geometry import CameraGeometry


def _noise(shape, rng, scale):
    return rng.normal(0.0, scale, shape)


def render(kind: str, geom: CameraGeometry, boxes=(), W=640, H=480, seed=0):
    """kind: 'tile' | 'pattern' | 'concrete'.  boxes: [(X, Y, rộng, cao, (B,G,R))]"""
    rng = np.random.default_rng(seed)
    xs = (np.arange(W) + 0.5) / W
    ys = (np.arange(H) + 0.5) / H
    xg, yg = np.meshgrid(xs, ys)
    v = (yg - 0.5) / geom.fy
    u = (xg - 0.5) / geom.fx
    t = geom.tilt
    down = math.sin(t) + v * math.cos(t)
    fwd = math.cos(t) - v * math.sin(t)
    ground = down > 1e-4
    s = np.where(ground, geom.h / np.maximum(down, 1e-4), 0.0)
    X = s * fwd
    Y = -s * u
    img = np.zeros((H, W, 3), np.float64)
    # tường phía trên đường chân trời
    img[:] = (110, 120, 128)
    img += _noise((H, W, 1), rng, 6)

    if kind in ("tile", "pattern"):
        ts = 0.6                                   # gạch 60 x 60 cm
        ix, iy = np.floor(X / ts), np.floor(Y / ts)
        fx, fy = X / ts - ix, Y / ts - iy
        tile_id = (ix * 7919 + iy * 104729).astype(np.int64)
        tone = ((tile_id * 2654435761) % 17 - 8).astype(np.float64)   # mỗi viên lệch màu nhẹ
        base = np.stack([200 + tone, 212 + tone, 222 + tone], axis=-1)
        if kind == "pattern":                      # gạch bông: hình tròn đỏ gạch + viền xanh
            r = np.hypot(fx - 0.5, fy - 0.5)
            ring = (r > 0.22) & (r < 0.30)
            disk = r <= 0.22
            base = np.where(disk[..., None], np.array([60, 80, 170.0]), base)
            base = np.where(ring[..., None], np.array([120, 130, 60.0]), base)
        speck = _noise((H, W, 1), rng, 5)          # vân đá hoa
        g = np.minimum(np.minimum(fx, 1 - fx), np.minimum(fy, 1 - fy)) * ts
        grout = g < 0.002                          # ron 4 mm
        col = base + speck
        col = np.where(grout[..., None], col - 55, col)
        # vệt phản sáng đèn trần trên gạch bóng
        for (hx, hy) in ((1.4, 0.3), (2.3, -0.5), (0.9, -0.4)):
            d2 = ((X - hx) / 0.25) ** 2 + ((Y - hy) / 0.12) ** 2
            col = np.where((d2 < 1)[..., None], 250 - 10 * d2[..., None], col)
    else:                                          # bê tông
        lo = rng.normal(0, 1, (8, 8))
        import cv2
        lo = cv2.resize(lo, (W, H), interpolation=cv2.INTER_CUBIC)
        gray = 138 + 10 * lo[..., None] + _noise((H, W, 1), rng, 7)
        col = np.concatenate([gray, gray + 2, gray + 4], axis=-1)
        for (sx, sy, a, b) in ((1.1, 0.25, 0.25, 0.15), (2.0, -0.4, 0.35, 0.2), (0.8, -0.2, 0.12, 0.10)):
            d2 = ((X - sx) / a) ** 2 + ((Y - sy) / b) ** 2
            col = np.where((d2 < 1)[..., None], col - 45, col)        # vết dầu / bẩn
        crack = np.abs(Y - 0.15 * np.sin(X * 3)) < 0.004
        col = np.where(crack[..., None], col - 60, col)               # vết nứt
        paint = (X > 1.5) & (X < 1.6)
        col = np.where(paint[..., None], np.array([30, 200, 230.0]), col)  # vạch sơn vàng ngang lối đi

    img = np.where(ground[..., None], col, img)

    # vật cản: mặt trước hộp đứng tại khoảng cách X0
    for (X0, Y0, w, h, color) in boxes:
        pts = [geom.image_of_ground(X0, Y0 + w / 2), geom.image_of_ground(X0, Y0 - w / 2)]
        x1, x2 = sorted(p[0] for p in pts)
        y_bot = geom.row_of_distance(X0)
        ang_top = math.atan2(geom.h - h, X0) - geom.tilt
        y_top = 0.5 + geom.fy * math.tan(ang_top)
        c0, c1 = int(max(0, x1 * W)), int(min(W, x2 * W))
        r0, r1 = int(max(0, y_top * H)), int(min(H, y_bot * H))
        if c1 > c0 and r1 > r0:
            img[r0:r1, c0:c1] = np.array(color, np.float64) + _noise((r1 - r0, c1 - c0, 1), rng, 8)
    return np.clip(img, 0, 255).astype(np.uint8)
