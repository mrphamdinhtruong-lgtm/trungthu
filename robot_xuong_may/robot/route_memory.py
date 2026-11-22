# -*- coding: utf-8 -*-
"""
DẠY – LẶP LỘ TRÌNH BẰNG CAMERA (visual teach & repeat) – KHÔNG DÁN LINE

Dùng khi trạm kế tiếp KHÔNG nhìn thấy được từ trạm trước (khuất sau dãy máy may, rẽ góc...).

  DẠY  : người vận hành lái tay robot một lần từ trạm A đến trạm B. Cứ mỗi ~25 cm (hoặc quay
         ~12°) robot lưu 1 "ẢNH MỐC" = các đặc trưng ORB của phần TRÊN ảnh (máy may, cột, tường,
         đèn trần...). Phần sàn ở đáy ảnh bị bỏ vì ron gạch / vân bê tông lặp lại dễ khớp nhầm.
  LẶP  : khi chạy tự động, so khớp ảnh hiện tại với ảnh mốc kế tiếp:
           - độ lệch ngang trung vị dx của các điểm khớp  -> lái (funnel lane, Chen & Birchfield 2006)
                 dx > 0 (cảnh lệch phải so với lúc dạy)  -> robot đang lệch trái -> quay phải
           - ảnh mốc kế tiếp khớp tốt hơn ảnh mốc hiện tại -> chuyển sang mốc kế tiếp
         Hết mốc cuối, hoặc YOLO thấy biển trạm đủ gần -> chuyển sang tiếp cận trạm.

File lộ trình: routes/leg_<từ>_to_<đến>.npz  (từ = 0 là vị trí xuất phát).
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from .common import clamp

log = logging.getLogger("route")
PROJECT_DIR = Path(__file__).resolve().parent.parent


@dataclass
class Keyframe:
    pts: np.ndarray            # (N, 2) tọa độ chuẩn hóa 0..1
    desc: np.ndarray           # (N, 32) mô tả ORB
    dist: float                # quãng đường (odometry) từ đầu chặng
    heading: float             # hướng (odometry, độ) từ đầu chặng


@dataclass
class RouteGuidance:
    ok: bool = False
    idx: int = 0
    n: int = 0
    matches: int = 0
    ex: float = 0.0            # độ lệch ngang chuẩn hóa (-1..1), > 0: cảnh lệch phải
    vx: float = 0.0
    wz: float = 0.0
    desired: float = 0.0       # hướng mong muốn (rad, + trái) cho bộ lái cục bộ
    remaining: float = float("inf")
    done: bool = False
    lost: bool = False


class FeatureExtractor:
    def __init__(self, width: int = 320, nfeatures: int = 400, floor_exclude: float = 0.4):
        import cv2
        self.cv2 = cv2
        self.width = width
        self.floor_exclude = floor_exclude
        self.orb = cv2.ORB_create(nfeatures=nfeatures, scaleFactor=1.2, nlevels=6,
                                  edgeThreshold=15, patchSize=31, fastThreshold=12)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self._mask = None

    def extract(self, frame) -> Tuple[np.ndarray, np.ndarray]:
        cv2 = self.cv2
        h0, w0 = frame.shape[:2]
        h = int(round(self.width * h0 / w0))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = cv2.resize(gray, (self.width, h), interpolation=cv2.INTER_AREA)
        gray = self.clahe.apply(gray)                   # chịu thay đổi ánh sáng xưởng
        if self._mask is None or self._mask.shape != gray.shape:
            self._mask = np.zeros_like(gray)
            self._mask[: int(h * (1.0 - self.floor_exclude)), :] = 255
        kps, desc = self.orb.detectAndCompute(gray, self._mask)
        if desc is None or not kps:
            return np.zeros((0, 2), np.float32), np.zeros((0, 32), np.uint8)
        pts = np.array([[k.pt[0] / self.width, k.pt[1] / h] for k in kps], np.float32)
        return pts, desc

    def match(self, desc_a: np.ndarray, desc_b: np.ndarray, ratio: float = 0.8) -> List[Tuple[int, int]]:
        if len(desc_a) < 2 or len(desc_b) < 2:
            return []
        out = []
        for pair in self.matcher.knnMatch(desc_a, desc_b, k=2):
            if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance:
                out.append((pair[0].queryIdx, pair[0].trainIdx))
        return out


class RouteMemory:
    def __init__(self, from_id: int, to_id: int, keyframes: Optional[List[Keyframe]] = None):
        self.from_id = int(from_id)
        self.to_id = int(to_id)
        self.keyframes: List[Keyframe] = keyframes or []

    @property
    def length(self) -> float:
        return self.keyframes[-1].dist if self.keyframes else 0.0

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        kfs = self.keyframes
        np.savez_compressed(
            path, from_id=self.from_id, to_id=self.to_id,
            counts=np.array([len(k.pts) for k in kfs], np.int32),
            pts=np.concatenate([k.pts for k in kfs]) if kfs else np.zeros((0, 2), np.float32),
            desc=np.concatenate([k.desc for k in kfs]) if kfs else np.zeros((0, 32), np.uint8),
            dist=np.array([k.dist for k in kfs], np.float32),
            heading=np.array([k.heading for k in kfs], np.float32))

    @classmethod
    def load(cls, path: Path) -> "RouteMemory":
        d = np.load(path, allow_pickle=False)
        kfs, i = [], 0
        for n, dist, hd in zip(d["counts"], d["dist"], d["heading"]):
            kfs.append(Keyframe(d["pts"][i:i + n], d["desc"][i:i + n], float(dist), float(hd)))
            i += n
        return cls(int(d["from_id"]), int(d["to_id"]), kfs)


class RouteRecorder:
    def __init__(self, from_id: int, to_id: int, extractor: FeatureExtractor,
                 spacing_m: float = 0.25, turn_deg: float = 12.0, min_features: int = 30):
        self.mem = RouteMemory(from_id, to_id)
        self.ex = extractor
        self.spacing = spacing_m
        self.turn = turn_deg
        self.min_features = min_features
        self.dist0: Optional[float] = None
        self.head0 = 0.0
        self.last_frame = None

    def feed(self, frame, odo_dist: float, heading_deg: float) -> bool:
        if frame is None:
            return False
        self.last_frame = frame
        if self.dist0 is None:
            self.dist0, self.head0 = odo_dist, heading_deg
        d, hd = odo_dist - self.dist0, heading_deg - self.head0
        kfs = self.mem.keyframes
        if kfs and abs(d - kfs[-1].dist) < self.spacing and abs(hd - kfs[-1].heading) < self.turn:
            return False
        pts, desc = self.ex.extract(frame)
        if len(pts) < self.min_features:
            return False
        kfs.append(Keyframe(pts, desc, d, hd))
        return True

    def finish(self, odo_dist: float, heading_deg: float) -> RouteMemory:
        """Thêm ảnh mốc cuối (vị trí trước trạm) rồi trả về lộ trình."""
        if self.last_frame is not None and self.dist0 is not None:
            pts, desc = self.ex.extract(self.last_frame)
            if len(pts) >= self.min_features:
                self.mem.keyframes.append(Keyframe(pts, desc, odo_dist - self.dist0, heading_deg - self.head0))
        return self.mem


class RouteFollower:
    def __init__(self, memory: RouteMemory, extractor: FeatureExtractor, cfg):
        self.mem = memory
        self.ex = extractor
        self.cfg = cfg
        self.half_fov = math.radians(cfg.CAMERA_HFOV_DEG) / 2.0
        self.t = 0
        self.odo0 = 0.0
        self._lost_since: Optional[float] = None

    @property
    def n(self) -> int:
        return len(self.mem.keyframes)

    def start(self, frame, odo_dist: float) -> int:
        """Định vị lại: tìm ảnh mốc giống ảnh hiện tại nhất (cho phép chạy tiếp giữa chặng)."""
        best, best_n = 0, -1
        if frame is not None and self.n:
            _, desc = self.ex.extract(frame)
            for i, kf in enumerate(self.mem.keyframes):
                m = len(self.ex.match(desc, kf.desc))
                if m > best_n:
                    best, best_n = i, m
            if best_n < self.cfg.ROUTE_MIN_MATCHES:
                best = 0
        self.t = min(best + 1, self.n - 1) if self.n > 1 else 0
        self.odo0 = odo_dist - (self.mem.keyframes[best].dist if self.n else 0.0)
        self._lost_since = None
        return best

    def _dx(self, pts, kf: Keyframe, matches) -> float:
        d = np.array([pts[i, 0] - kf.pts[j, 0] for i, j in matches], np.float64)
        med = float(np.median(d))
        inl = d[np.abs(d - med) < 0.08]                    # loại điểm khớp sai
        return float(np.median(inl)) if inl.size else med

    def update(self, frame, odo_dist: float, now: Optional[float] = None) -> RouteGuidance:
        c = self.cfg
        now = time.monotonic() if now is None else now
        g = RouteGuidance(n=self.n, idx=self.t)
        if frame is None or self.n == 0:
            return g
        pts, desc = self.ex.extract(frame)
        kfs = self.mem.keyframes
        odo = odo_dist - self.odo0
        cur = self.t
        nxt = min(cur + 1, self.n - 1)
        m_cur = self.ex.match(desc, kfs[cur].desc)
        if nxt != cur:
            m_nxt = self.ex.match(desc, kfs[nxt].desc)
            spacing = c.ROUTE_KF_SPACING_M
            near = odo >= kfs[cur].dist - 0.5 * spacing
            if (near and len(m_nxt) >= max(c.ROUTE_MIN_MATCHES, 0.9 * len(m_cur))) or \
               (odo >= kfs[cur].dist + spacing and len(m_nxt) >= c.ROUTE_MIN_MATCHES):
                self.t = cur = nxt
                m_cur = m_nxt
        g.idx = self.t
        g.matches = len(m_cur)
        g.remaining = max(0.0, kfs[-1].dist - odo)
        if len(m_cur) < c.ROUTE_MIN_MATCHES:
            if self._lost_since is None:
                self._lost_since = now
            g.lost = (now - self._lost_since) > c.ROUTE_LOST_S
            return g                                       # đứng chờ khớp lại (vx = wz = 0)
        self._lost_since = None
        ex = clamp(2.0 * self._dx(pts, kfs[cur], m_cur), -1.0, 1.0)
        g.ok = True
        g.ex = ex
        g.wz = clamp(-c.ROUTE_K_W * ex, -c.W_MAX, c.W_MAX)
        g.desired = -ex * self.half_fov
        rotating = cur > 0 and abs(kfs[cur].dist - kfs[cur - 1].dist) < 0.05 and \
            abs(kfs[cur].heading - kfs[cur - 1].heading) > 5.0
        g.vx = 0.0 if rotating else c.ROUTE_V * clamp(1.0 - abs(ex) / max(1e-3, c.EX_TURN_ONLY), 0.0, 1.0)
        g.done = (self.t == self.n - 1 and abs(ex) < 0.15 and odo >= kfs[-1].dist - c.ROUTE_KF_SPACING_M)
        return g


class RouteManager:
    """Quản lý các chặng đã dạy: ghi (dạy), nạp và chạy theo (lặp)."""

    def __init__(self, cfg, route_dir: Optional[str] = None):
        self.cfg = cfg
        d = Path(route_dir or cfg.ROUTE_DIR)
        self.dir = d if d.is_absolute() else PROJECT_DIR / d
        self._ex: Optional[FeatureExtractor] = None
        self.recorder: Optional[RouteRecorder] = None
        self.follower: Optional[RouteFollower] = None
        self.leg: Optional[Tuple[int, int]] = None

    @property
    def extractor(self) -> FeatureExtractor:
        if self._ex is None:
            c = self.cfg
            self._ex = FeatureExtractor(320, c.ROUTE_FEATURES, c.ROUTE_FLOOR_EXCLUDE)
        return self._ex

    def path(self, frm: int, to: int) -> Path:
        return self.dir / f"leg_{int(frm)}_to_{int(to)}.npz"

    def has(self, frm: int, to: int) -> bool:
        return self.path(frm, to).exists()

    def legs(self) -> List[Tuple[int, int]]:
        out = []
        for p in sorted(self.dir.glob("leg_*_to_*.npz")):
            try:
                a, b = p.stem[4:].split("_to_")
                out.append((int(a), int(b)))
            except ValueError:
                pass
        return out

    # ---- DẠY ----
    @property
    def teaching(self) -> bool:
        return self.recorder is not None

    def start_teach(self, frm: int, to: int) -> None:
        c = self.cfg
        self.recorder = RouteRecorder(frm, to, self.extractor, c.ROUTE_KF_SPACING_M, c.ROUTE_KF_TURN_DEG)
        log.info("Bắt đầu dạy chặng %d -> %d", frm, to)

    def teach_feed(self, frame, odo_dist: float, heading_deg: float) -> None:
        if self.recorder is not None:
            self.recorder.feed(frame, odo_dist, heading_deg)

    def teach_info(self) -> Tuple[int, float]:
        if self.recorder is None:
            return 0, 0.0
        k = self.recorder.mem.keyframes
        return len(k), (k[-1].dist if k else 0.0)

    def stop_teach(self, save: bool, odo_dist: float = 0.0, heading_deg: float = 0.0):
        rec, self.recorder = self.recorder, None
        if rec is None or not save:
            return None
        mem = rec.finish(odo_dist, heading_deg)
        if len(mem.keyframes) < 2:
            log.warning("Lộ trình quá ngắn – không lưu")
            return None
        p = self.path(mem.from_id, mem.to_id)
        mem.save(p)
        log.info("Đã lưu %s: %d ảnh mốc, %.2f m", p.name, len(mem.keyframes), mem.length)
        return mem

    # ---- LẶP ----
    def start_leg(self, frm: int, to: int, frame, odo_dist: float) -> bool:
        self.follower, self.leg = None, None
        if not self.has(frm, to):
            return False
        mem = RouteMemory.load(self.path(frm, to))
        self.follower = RouteFollower(mem, self.extractor, self.cfg)
        start = self.follower.start(frame, odo_dist)
        self.leg = (frm, to)
        log.info("Chạy theo chặng %d -> %d (%d mốc, bắt đầu ở mốc %d)", frm, to, self.follower.n, start)
        return True

    def follow(self, frame, odo_dist: float, now: Optional[float] = None) -> Optional[RouteGuidance]:
        if self.follower is None:
            return None
        return self.follower.update(frame, odo_dist, now)

    def stop_leg(self) -> None:
        self.follower, self.leg = None, None
