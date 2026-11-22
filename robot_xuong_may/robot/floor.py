# -*- coding: utf-8 -*-
"""
NHẬN DIỆN MẶT SÀN TRỐNG BẰNG CAMERA (free-space) – không cần dán line, không cần huấn luyện.

Ý tưởng (Ulrich & Nourbakhsh, "Appearance-Based Obstacle Detection with Monocular Color
Vision", AAAI 2000): vùng sàn NGAY TRƯỚC robot (vùng tham chiếu) được coi là sàn trống ->
học "màu sàn" bằng histogram 3D (L, a, b). Điểm ảnh nào có màu KHÔNG giống sàn là vật cản.
Quét từng cột ảnh từ dưới lên, điểm đầu tiên không phải sàn = chân vật cản -> đổi ra mét
nhờ hình học camera => "cảm biến khoảng cách ảo" giống laser quét 2D.

Xử lý riêng cho từng loại sàn:
  * GẠCH / ĐÁ HOA: ron gạch mảnh bị lọc trung vị xóa; gạch nhiều màu (gạch bông) vẫn đúng vì
    histogram học được nhiều màu cùng lúc; vệt phản sáng đèn trên gạch bóng (rất sáng, ít màu)
    được coi là sàn.
  * BÊ TÔNG / EPOXY: vân bê tông được làm mờ; vết bẩn, vết nứt, vạch sơn PHẲNG bị loại bằng
    bộ lọc "chiều cao": chỉ vật nhô cao hơn FLOOR_MIN_OBST_HEIGHT_M mới tính là vật cản.
  * Đổi loại sàn khi đang chạy (gạch -> bê tông): histogram tự thích nghi dần khi robot tiến.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .geometry import CameraGeometry

log = logging.getLogger("floor")

# median : lọc trung vị (xóa ron gạch, vân bê tông)    l_bins/ab_bins : số bin histogram độ sáng / màu
# l_tol  : dung sai độ sáng (số bin) – lớn = chịu bóng đổ, vết bẩn   glossy : coi vệt phản sáng là sàn
# tex_k, tex_min : ngưỡng độ vân (gradient) = trung bình + k·σ + min
FLOOR_PRESETS = {
    "tile":     dict(median=5, l_bins=8, ab_bins=32, l_tol=2, glossy=True,  tex_k=4.0, tex_min=22.0),
    "concrete": dict(median=5, l_bins=8, ab_bins=32, l_tol=2, glossy=False, tex_k=3.0, tex_min=14.0),
    "auto":     dict(median=5, l_bins=8, ab_bins=32, l_tol=2, glossy=True,  tex_k=3.5, tex_min=18.0),
}


@dataclass
class FreeSpace:
    """Kết quả quét sàn: mỗi "tia" (cột ảnh) cho khoảng cách tới vật cản gần nhất."""
    valid: bool = False
    dist: np.ndarray = field(default_factory=lambda: np.zeros(0))   # khoảng cách tiến X (m)
    lat: np.ndarray = field(default_factory=lambda: np.zeros(0))    # lệch ngang Y (m, + trái)
    xs: np.ndarray = field(default_factory=lambda: np.zeros(0))     # tâm cột (0..1)
    rows: np.ndarray = field(default_factory=lambda: np.zeros(0))   # hàng chân vật cản (0..1), nan nếu trống
    max_range: float = 3.0
    mask: Optional[np.ndarray] = None                                # mặt nạ sàn nhỏ (để hiển thị)
    learned: bool = False

    def points(self) -> Tuple[np.ndarray, np.ndarray]:
        """Tọa độ (X, Y) các điểm chân vật cản (bỏ các tia không gặp vật)."""
        if not self.valid or self.dist.size == 0:
            return np.zeros(0), np.zeros(0)
        hit = self.dist < self.max_range - 1e-6
        return self.dist[hit], self.lat[hit]

    def nearest_in_lane(self, half_width: float) -> float:
        X, Y = self.points()
        inlane = np.abs(Y) < half_width
        return float(X[inlane].min()) if np.any(inlane) else float("inf")


class FloorDetector:
    def __init__(self, cfg, geom: Optional[CameraGeometry] = None):
        self.cfg = cfg
        self.geom = geom or CameraGeometry.from_config(cfg)
        preset = FLOOR_PRESETS.get(str(cfg.FLOOR_TYPE).lower(), FLOOR_PRESETS["auto"])
        self.p = dict(preset)
        self.W = int(cfg.FLOOR_PROC_WIDTH)
        self.nb = int(cfg.FLOOR_BINS)
        if self.W % self.nb:
            self.W = self.nb * max(1, round(self.W / self.nb))
        self.max_range = float(cfg.FLOOR_MAX_RANGE_M)
        self._shape = None
        self.reset()

    def reset(self) -> None:
        lb, ab = self.p["l_bins"], self.p["ab_bins"]
        self.hist = np.zeros(lb * ab * ab, np.float64)
        self.hist_s = None
        self.ready = False
        self.ref_like = 1.0
        self.tex_mu = 0.0
        self.tex_sd = 1.0
        self._force_learn = True
        self.learn_count = 0

    def learn_now(self) -> None:
        """Người vận hành yêu cầu học lại màu sàn (phía trước robot phải trống)."""
        self._force_learn = True
        self.hist[:] = 0
        self.ready = False

    # ------------------------------------------------------------------
    def _prepare(self, H: int) -> None:
        """Tính trước các bảng phụ thuộc kích thước ảnh."""
        g = self.geom
        c = self.cfg
        W, nb = self.W, self.nb
        self.H = H
        ys = (np.arange(H) + 1.0) / H                         # mép dưới mỗi hàng
        dist = np.array([g.ground_distance(min(y, 1.0)) for y in ys])
        self.row_dist = dist
        y_top = g.row_of_distance(self.max_range)
        self.r_top = int(np.clip(math.floor(y_top * H), 0, H - 1))
        # số hàng tối thiểu của vật cao FLOOR_MIN_OBST_HEIGHT_M đứng tại khoảng cách của hàng đó
        hmin = float(c.FLOOR_MIN_OBST_HEIGHT_M)
        with np.errstate(divide="ignore", invalid="ignore"):
            req = np.ceil(g.fy * hmin / dist * H)
        req = np.where(np.isfinite(req), req, H)
        req = np.maximum(req, 3)
        self.req_rows = np.minimum(req, np.arange(H) + 1)    # không đòi hơn số hàng phía trên
        bot = 1.0 - float(c.FLOOR_BOTTOM_CROP)

        def region(rows, cols):
            (r0, r1), (c0, c1) = rows, cols
            return (slice(int(r0 * bot * H), max(int(r0 * bot * H) + 2, int(r1 * bot * H))),
                    slice(int(c0 * W), max(int(c0 * W) + 2, int(c1 * W))))
        self.ref = region(c.FLOOR_REF_ROWS, c.FLOOR_REF_COLS)          # học liên tục khi chạy
        self.ref_init = region(c.FLOOR_INIT_ROWS, c.FLOOR_INIT_COLS)   # học lần đầu / nhấn F (vùng lớn)
        self.bottom_limit = max(1, int(round(bot * H)))       # bỏ phần thân robot lọt vào khung
        xs = (np.arange(nb) + 0.5) / nb
        self.xs = xs
        self._shape = (H, W)

    def _quantize(self, lab: np.ndarray) -> np.ndarray:
        lb, ab = self.p["l_bins"], self.p["ab_bins"]
        qL = (lab[..., 0].astype(np.int32) * lb) >> 8
        qa = (lab[..., 1].astype(np.int32) * ab) >> 8
        qb = (lab[..., 2].astype(np.int32) * ab) >> 8
        return (qL * ab + qa) * ab + qb

    def _smooth_hist(self) -> np.ndarray:
        """Làm mờ histogram: ±l_tol bin theo độ sáng, ±1 bin theo màu (dung sai)."""
        lb, ab, lt = self.p["l_bins"], self.p["ab_bins"], int(self.p["l_tol"])
        h = self.hist.reshape(lb, ab, ab)
        hp = np.pad(h, ((lt, lt), (1, 1), (1, 1)), mode="constant")
        s = np.zeros_like(h)
        for dl in range(2 * lt + 1):
            for da in (0, 1, 2):
                for db in (0, 1, 2):
                    w = 1.0 if (dl == lt and da == 1 and db == 1) else 0.5
                    s += w * hp[dl:dl + lb, da:da + ab, db:db + ab]
        return s.ravel()

    # ------------------------------------------------------------------
    def process(self, frame_bgr, learn: bool = False) -> FreeSpace:
        import cv2
        if frame_bgr is None:
            return FreeSpace(valid=False)
        fh, fw = frame_bgr.shape[:2]
        H = max(8, int(round(self.W * fh / fw)))
        if self._shape != (H, self.W):
            self._prepare(H)
        small = cv2.resize(frame_bgr, (self.W, H), interpolation=cv2.INTER_AREA)
        k = self.p["median"]
        blur = cv2.medianBlur(small, k) if k >= 3 else small
        lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
        Lf = lab[..., 0].astype(np.float32)
        gx = cv2.Sobel(Lf, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(Lf, cv2.CV_32F, 0, 1, ksize=3)
        grad = cv2.blur(cv2.magnitude(gx, gy), (5, 5))
        idx = self._quantize(lab)

        # ---- học màu sàn từ vùng tham chiếu ----
        if learn or self._force_learn or not self.ready:
            self._learn(idx, grad, Lf)
        if not self.ready:
            return FreeSpace(valid=False, max_range=self.max_range)

        like = self.hist_s[idx]
        floor = like >= self.cfg.FLOOR_LIKE_RATIO * self.ref_like
        floor &= grad <= self.tex_mu + self.p["tex_k"] * self.tex_sd + self.p["tex_min"]
        if self.p["glossy"]:
            floor |= self._flat_highlights(lab, floor)
        fm = floor.astype(np.uint8)
        fm = cv2.morphologyEx(fm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        fm = cv2.morphologyEx(fm, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        fm[self.bottom_limit:, :] = 1
        return self._scan(fm)

    def _flat_highlights(self, lab: np.ndarray, floor: np.ndarray) -> np.ndarray:
        """Vệt phản sáng đèn trên gạch bóng: rất sáng, gần như không màu và PHẲNG (thấp trong ảnh).
        Vật màu trắng ĐỨNG (thùng trắng, áo trắng) cao hơn -> không bị coi là sàn."""
        import cv2
        a = lab[..., 1].astype(np.int16) - 128
        b = lab[..., 2].astype(np.int16) - 128
        hi = ((lab[..., 0] > 215) & (np.abs(a) + np.abs(b) < 24) & ~floor).astype(np.uint8)
        out = np.zeros_like(floor)
        if not hi.any():
            return out
        n, labels, stats, _ = cv2.connectedComponentsWithStats(hi, connectivity=8)
        for i in range(1, n):
            top, h = stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_HEIGHT]
            bottom = min(self.H - 1, top + h - 1)
            if bottom >= self.r_top and h < 0.8 * self.req_rows[bottom]:
                out |= labels == i
        return out

    def _learn(self, idx, grad, Lf) -> None:
        init = self._force_learn or not self.ready
        rs, cs = self.ref_init if init else self.ref
        ref_idx = idx[rs, cs].ravel()
        if ref_idx.size < 20:
            return
        ref_L = Lf[rs, cs]
        if self.ready and not self._force_learn and float(ref_L.std()) > 45.0:
            return                                           # vùng tham chiếu quá loang lổ -> bỏ qua
        h = np.bincount(ref_idx, minlength=self.hist.size).astype(np.float64)
        h /= max(1.0, h.sum())
        if not self.ready or self._force_learn:
            self.hist = h
        else:
            a = float(self.cfg.FLOOR_LEARN_RATE)
            self.hist = (1.0 - a) * self.hist + a * h
        self.hist_s = self._smooth_hist()
        self.ref_like = float(np.median(self.hist_s[ref_idx])) or 1e-6
        g = grad[rs, cs]
        mu, sd = float(g.mean()), float(g.std())
        if not self.ready or self._force_learn:
            self.tex_mu, self.tex_sd = mu, sd
        else:
            self.tex_mu += 0.1 * (mu - self.tex_mu)
            self.tex_sd += 0.1 * (sd - self.tex_sd)
        self.ready = True
        self._force_learn = False
        self.learn_count += 1

    def _scan(self, fm: np.ndarray) -> FreeSpace:
        H, W, nb = self.H, self.W, self.nb
        nonfloor = 1 - fm
        frac = nonfloor.reshape(H, nb, W // nb).mean(axis=2)       # tỉ lệ "không phải sàn" mỗi ô
        O = frac > 0.5
        run = np.zeros((H, nb), np.int32)                         # số hàng vật cản liên tiếp hướng lên
        run[0] = O[0]
        for r in range(1, H):
            run[r] = np.where(O[r], run[r - 1] + 1, 0)
        cand = run >= self.req_rows[:, None]
        cand[: self.r_top] = False
        has = cand.any(axis=0)
        bottom = H - 1 - np.argmax(cand[::-1], axis=0)             # hàng thấp nhất (gần nhất) có vật
        dist = np.where(has, self.row_dist[bottom], self.max_range)
        dist = np.minimum(dist, self.max_range)
        rows = np.where(has, (bottom + 1.0) / H, np.nan)
        g = self.geom
        lat = np.array([g.lateral(x, d, y if np.isfinite(y) else None) if np.isfinite(d) else 0.0
                        for x, d, y in zip(self.xs, dist, rows)])
        return FreeSpace(valid=True, dist=dist, lat=lat, xs=self.xs.copy(), rows=rows,
                         max_range=self.max_range, mask=fm, learned=self.ready)
