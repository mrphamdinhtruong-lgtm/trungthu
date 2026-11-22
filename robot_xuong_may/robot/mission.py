# -*- coding: utf-8 -*-
"""
MÁY TRẠNG THÁI NHIỆM VỤ – đi lần lượt các trạm, dừng chờ bỏ rác, tránh va chạm.

                 ┌─ có lộ trình đã dạy ─> FOLLOW ──thấy biển trạm──┐
  IDLE ──START──>┤                        (hết mốc / mất dấu)      │
                 └─ không ──────────────> SEARCH ──thấy trạm──> APPROACH ──đủ gần──> WAIT (20 s)
                                            │                       │                 │
                                            └─quá thời gian─> LOST  └─mất dấu─> SEARCH│
                    trạm kế tiếp <──────────── (DEPART: lùi, tùy chọn) <──────────────┘

  FOLLOW / APPROACH dùng BỘ LÁI CỤC BỘ: lách vật cản ở xa theo khe trống trên sàn (camera),
  vật cản trong làn đường gần hơn OBST_STOP_DISTANCE_M -> DỪNG.

  Bất kỳ trạng thái di chuyển nào + vật cản trong hành lang -> OBSTACLE (dừng),
  hết vật cản OBST_CLEAR_S giây -> quay lại trạng thái trước đó.
  ESTOP (dừng khẩn, chốt) và FAULT (mất kết nối, lỗi camera, pin yếu) ưu tiên cao nhất.

Lớp này KHÔNG truy cập phần cứng: nhận danh sách Detection + thời gian, trả về lệnh
(vx, vy, wz) và nội dung hiển thị -> kiểm thử được hoàn toàn (tests/test_mission.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

from .common import Detection, clamp, focal_norm, wrap180
from .geometry import CameraGeometry
from .navigator import LocalPlanner, ObstacleMemory


class St(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    FOLLOW = "FOLLOW"
    APPROACH = "APPROACH"
    WAIT = "WAIT"
    DEPART = "DEPART"
    OBSTACLE = "OBSTACLE"
    LOST = "LOST"
    DONE = "DONE"
    MANUAL = "MANUAL"
    ESTOP = "ESTOP"
    FAULT = "FAULT"


# Trạng thái robot tự di chuyển (cần kiểm tra vật cản)
MOVING_STATES = (St.SEARCH, St.FOLLOW, St.APPROACH, St.DEPART)


@dataclass
class MissionOutput:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    state: St = St.IDLE
    station: Optional[int] = None          # trạm đích / trạm đang đứng
    title: str = ""
    subtitle: str = ""
    level: str = "idle"                    # idle | move | wait | warn | alarm | manual
    countdown: Optional[float] = None
    distance: Optional[float] = None
    target_det: Optional[Detection] = None
    obstacle_det: Optional[Detection] = None
    beep: Optional[str] = None             # sự kiện còi: start | arrive | obstacle | alarm
    drive: bool = False                    # True: gửi tốc độ xuống Arduino, False: lệnh dừng
    route: List[int] = field(default_factory=list)
    route_idx: int = 0
    lap: int = 0
    nav: str = ""                          # trạng thái bộ lái: clear | avoid | blocked | none
    nav_theta: Optional[float] = None      # hướng khe trống được chọn (rad)
    route_kf: Optional[str] = None         # tiến độ lộ trình đã dạy, vd "12/40"
    obstacle_dist: Optional[float] = None  # khoảng cách vật cản gần nhất trong làn (m)


class Mission:
    def __init__(self, cfg, routes=None):
        self.cfg = cfg
        self.routes = routes                    # RouteManager (có .has(từ, đến)) hoặc None
        self.geom = CameraGeometry.from_config(cfg)
        self.planner = LocalPlanner(cfg)
        self.memory = ObstacleMemory(self.geom, max_range=cfg.FLOOR_MAX_RANGE_M)
        self._navpts = None
        self.half_width = cfg.ROBOT_HALF_WIDTH_M + cfg.NAV_SAFETY_MARGIN_M
        self.prev_station = 0                   # trạm vừa rời (0 = vị trí xuất phát)
        self._free = None
        self._route = None
        self._route_reset = False
        self._station_dist: Optional[float] = None
        self._last_desired = 0.0
        self._last_look = 1.0
        self._nav_status = ""
        self._motion_theta = 0.0                # hướng chuyển động thực (mecanum có thể đi chéo)
        self._last_trans = 0.0                  # tốc độ tịnh tiến của lệnh trước (m/s)
        self.route: List[int] = list(cfg.ROUTE)
        if not self.route:
            raise ValueError("ROUTE trong config.py đang rỗng")
        for sid in self.route:
            if sid not in cfg.STATIONS:
                raise ValueError(f"Trạm {sid} trong ROUTE chưa khai báo trong STATIONS")
        self.f_norm = focal_norm(cfg.CAMERA_VFOV_DEG)
        self.route_idx = 0
        self.lap = 0
        self.state = St.IDLE
        self.visits: Dict[int, int] = {sid: 0 for sid in cfg.STATIONS}
        self._now = 0.0
        self._last_update: Optional[float] = None
        self._t_state = 0.0
        self._prev_state: Optional[St] = None
        self._beep: Optional[str] = None
        self._fault_msg = ""
        self._estop_src = ""
        self._info_msg = ""
        self._info_until = 0.0
        # vật cản
        self._obst_frames = 0
        self._last_obst_t = -1e9
        # tìm trạm
        self._confirm = 0
        self._search_time = 0.0
        self._search_phase = "sweep"
        self._search_dir_override: Optional[int] = None
        # tiếp cận
        self._arrive_cnt = 0
        self._last_seen = 0.0
        self._smooth: Optional[List[float]] = None
        self._last_ex = 0.0
        self._last_cmd = (0.0, 0.0, 0.0)
        # chờ / rời trạm
        self._wait_end = 0.0
        self._skip = False
        self._depart_time = 0.0
        self._hold_yaw: Optional[float] = None
        # tay
        self._manual_cmd = (0.0, 0.0, 0.0)
        self._manual_t = -1e9

    # ------------------------------------------------------------------
    # Thông tin trạm
    # ------------------------------------------------------------------
    @property
    def target(self) -> int:
        return self.route[self.route_idx]

    def station_cfg(self, sid: Optional[int] = None) -> dict:
        return self.cfg.STATIONS[self.target if sid is None else sid]

    def station_name(self, sid: Optional[int] = None) -> str:
        return self.station_cfg(sid).get("name", f"TRẠM {self.target if sid is None else sid}")

    # ------------------------------------------------------------------
    # Lệnh từ người vận hành
    # ------------------------------------------------------------------
    def start(self) -> None:
        """START: chạy tự động (từ IDLE / LOST / DONE / MANUAL)."""
        if self.state in (St.IDLE, St.LOST, St.DONE, St.MANUAL):
            if self.state == St.DONE:
                self.route_idx = 0
            self._beep = "start"
            if self.state == St.LOST:
                self._enter(St.SEARCH)
            else:
                self._start_leg()

    def has_route(self) -> bool:
        return bool(self.cfg.ROUTE_ENABLE and self.routes is not None
                    and self.routes.has(self.prev_station, self.target))

    def _start_leg(self) -> None:
        """Bắt đầu chặng tới trạm đích: có lộ trình đã dạy -> FOLLOW, không -> quay tìm biển."""
        self._enter(St.FOLLOW if self.has_route() else St.SEARCH)

    def consume_route_reset(self) -> bool:
        """Chương trình chính hỏi: có cần nạp/định vị lại lộ trình chặng hiện tại không."""
        r, self._route_reset = self._route_reset, False
        return r

    def mark_at_station(self, sid: int) -> None:
        """Sau khi DẠY xong chặng tới trạm sid: coi như robot đang đứng ở trạm sid,
        trạm đích chuyển sang trạm kế tiếp trong lộ trình."""
        self.prev_station = int(sid)
        if sid in self.route:
            i = self.route.index(sid) + 1
            self.route_idx = i % len(self.route)

    def pause(self) -> None:
        """STOP/PAUSE: về IDLE, giữ nguyên trạm đích để chạy tiếp."""
        if self.state not in (St.ESTOP, St.FAULT, St.IDLE):
            self._enter(St.IDLE)

    def toggle_manual(self) -> None:
        if self.state == St.MANUAL:
            self._enter(St.IDLE)
        elif self.state not in (St.ESTOP, St.FAULT):
            self._enter(St.MANUAL)

    def set_manual(self, vx: float, vy: float, wz: float, now: float) -> None:
        self._manual_cmd = (vx, vy, wz)
        self._manual_t = now

    def skip_wait(self) -> None:
        """Bỏ qua thời gian chờ còn lại tại trạm."""
        if self.state == St.WAIT:
            self._skip = True

    def trigger_estop(self, source: str = "") -> None:
        if self.state != St.ESTOP:
            self._estop_src = source
            self._beep = "alarm"
            self._enter(St.ESTOP)

    def reset(self) -> None:
        """Reset sau dừng khẩn / lỗi (chương trình chính kiểm tra điều kiện an toàn trước)."""
        if self.state in (St.ESTOP, St.FAULT):
            self._enter(St.IDLE)

    # ------------------------------------------------------------------
    # Chuyển trạng thái
    # ------------------------------------------------------------------
    def _enter(self, st: St) -> None:
        now = self._now
        self.state = st
        self._t_state = now
        if st in (St.IDLE, St.MANUAL, St.ESTOP):
            # robot có thể bị bê đi chỗ khác (odometry không biết) -> quên vật cản đã nhớ
            self.memory.reset()
        if st == St.SEARCH:
            self._confirm = 0
            self._search_time = 0.0
            hint = self.station_cfg().get("heading_deg")
            self._search_phase = "align" if hint is not None else "sweep"
        elif st == St.FOLLOW:
            self._confirm = 0
            self._route_reset = True
            self._route = None
        elif st == St.APPROACH:
            self._arrive_cnt = 0
            self._last_seen = now
            self._smooth = None
            self._last_cmd = (0.0, 0.0, 0.0)
        elif st == St.WAIT:
            self._wait_end = now + float(self.station_cfg().get("wait_s", 20.0))
            self._skip = False
            self.visits[self.target] = self.visits.get(self.target, 0) + 1
            self._beep = "arrive"
        elif st == St.DEPART:
            self._depart_time = 0.0
        elif st == St.OBSTACLE:
            self._beep = "obstacle"
        elif st == St.LOST:
            self._beep = "alarm"
        elif st == St.FAULT:
            self._beep = "alarm"

    def _advance_route(self, yaw: Optional[float]) -> None:
        self.prev_station = self.target
        self.route_idx += 1
        self._search_dir_override = None
        if self.route_idx >= len(self.route):
            if self.cfg.ROUTE_LOOP:
                self.route_idx = 0
                self.lap += 1
            else:
                self.route_idx = len(self.route) - 1
                self._enter(St.DONE)
                return
        if self.cfg.DEPART_REVERSE_S > 0:
            self._hold_yaw = yaw
            self._enter(St.DEPART)
        else:
            self._start_leg()

    # ------------------------------------------------------------------
    # Xử lý kết quả nhận diện
    # ------------------------------------------------------------------
    def _target_det(self, dets: Sequence[Detection]) -> Optional[Detection]:
        cls = self.station_cfg()["class"]
        best = None
        for d in dets:
            if d.cls == cls and d.conf >= self.cfg.STATION_CONF:
                if best is None or d.conf > best.conf:
                    best = d
        return best

    def _obstacle_geom(self, d: Detection) -> Tuple[float, float, float]:
        """Khung YOLO -> (khoảng cách chân vật X, lệch ngang mép trái Y1, mép phải Y2) theo mét."""
        X = self.ground_distance(min(d.y2, 1.0))
        if not math.isfinite(X):
            return X, 0.0, 0.0
        y = min(d.y2, 1.0)
        return X, self.geom.lateral(d.x1, X, y), self.geom.lateral(d.x2, X, y)

    def _obstacle_det(self, dets: Sequence[Detection]) -> Optional[Detection]:
        """Vật cản YOLO nằm TRONG LÀN ĐƯỜNG của robot (bề rộng thật) và gần hơn khoảng dừng."""
        c = self.cfg
        worst = None
        for d in dets:
            if d.cls not in c.OBSTACLE_CLASSES or d.conf < c.OBST_CONF:
                continue
            X, Yl, Yr = self._obstacle_geom(d)
            if not math.isfinite(X):
                continue
            th = self._motion_theta
            ct, st = math.cos(th), math.sin(th)
            hit = False
            for k in range(7):                    # các điểm dọc mép dưới khung vật cản
                Y = Yl + (Yr - Yl) * k / 6.0
                along = X * ct + Y * st
                lat = abs(X * st - Y * ct)
                if lat < self.half_width and 0 < along <= c.OBST_STOP_DISTANCE_M:
                    hit = True
                    break
            if hit and (worst is None or d.y2 > worst.y2):
                worst = d
        return worst

    def _yolo_points(self, dets: Sequence[Detection]):
        """Vật cản YOLO ở xa -> các điểm (X, Y) cho bộ lái cục bộ tránh từ sớm."""
        import numpy as np
        xs, ys = [], []
        for d in dets:
            if d.cls in self.cfg.OBSTACLE_CLASSES and d.conf >= self.cfg.OBST_CONF:
                X, Yl, Yr = self._obstacle_geom(d)
                if math.isfinite(X) and X < self.cfg.NAV_LOOKAHEAD_M + 1.0:
                    for Y in np.linspace(Yl, Yr, 5):
                        xs.append(X)
                        ys.append(Y)
        return np.array(xs), np.array(ys)

    def _free_stop_distance(self, state: St) -> float:
        """Khoảng dừng theo sàn. Gần trạm thì rút ngắn để chân biển / bàn / thùng của
        chính trạm không bị coi là vật cản."""
        d = self.cfg.OBST_STOP_DISTANCE_M
        if state == St.APPROACH and self._station_dist is not None:
            d = min(d, max(0.2, self._station_dist - 0.25))
        if state == St.FOLLOW and self._route is not None and math.isfinite(self._route.remaining):
            d = min(d, max(0.25, self._route.remaining - 0.1))
        return d

    def _build_nav_points(self, dets, pose):
        """Gộp điểm vật cản: sàn (camera) + YOLO ở xa + bộ nhớ ngắn hạn (vật đã ra khỏi tầm nhìn)."""
        if self._free is None:
            return None
        import numpy as np
        from .floor import FreeSpace
        X, Y = self._free.points()
        Xy, Yy = self._yolo_points(dets)
        X, Y = np.concatenate([X, Xy]), np.concatenate([Y, Yy])
        if pose is not None:
            self.memory.update(X, Y, pose, self._now)
            Xm, Ym = self.memory.recall(pose)
            X, Y = np.concatenate([X, Xm]), np.concatenate([Y, Ym])
        return FreeSpace(valid=True, dist=X, lat=Y, max_range=self._free.max_range + 10.0)

    def _plan(self, vx, vy, wz, desired, look, state, dets):
        """Cho lệnh đi qua bộ lái cục bộ."""
        if self._navpts is None:
            return vx, vy, wz, "none"
        if not self.cfg.NAV_AVOID_ENABLE:
            return vx, vy, wz, "clear"
        self._last_desired, self._last_look = desired, look
        res = self.planner.adjust(vx, vy, wz, self._navpts, desired, look, self._free_stop_distance(state))
        return res.vx, res.vy, res.wz, res.status

    def ground_distance(self, y_bottom: float) -> float:
        """Khoảng cách (m) tới điểm chạm sàn của vật có mép dưới ở y_bottom (0..1),
        giả thiết sàn phẳng, camera cao CAMERA_HEIGHT_M, chúc xuống CAMERA_TILT_DEG."""
        c = self.cfg
        ang = math.radians(c.CAMERA_TILT_DEG) + math.atan((y_bottom - 0.5) / self.f_norm)
        if ang <= 1e-3:
            return float("inf")            # trên đường chân trời -> ở rất xa
        return c.CAMERA_HEIGHT_M / math.tan(ang)

    def distance_from_det(self, det: Detection, sid: Optional[int] = None) -> float:
        marker_h = float(self.station_cfg(sid).get("marker_h_m", 0.2))
        return self.f_norm * marker_h / max(det.h, 1e-3)

    # ------------------------------------------------------------------
    # Vòng cập nhật chính (gọi mỗi khung hình)
    # ------------------------------------------------------------------
    @property
    def wants_route(self) -> bool:
        """Có cần chương trình chính chạy bộ so khớp ảnh mốc ở vòng lặp này không."""
        return self.state == St.FOLLOW or (self.state == St.OBSTACLE and self._prev_state == St.FOLLOW)

    def update(self, now: float, dets: Sequence[Detection], yaw_deg: Optional[float] = None,
               faults: Optional[Dict[str, str]] = None, free=None, route=None,
               pose=None) -> MissionOutput:
        """free : FreeSpace (sàn trống) hoặc None
        route: RouteGuidance (lộ trình đã dạy) hoặc None
        pose : (x, y, θ) từ odometry – để nhớ vật cản đã ra khỏi tầm nhìn"""
        self._now = now
        self._free = free if (free is not None and getattr(free, "valid", False)) else None
        self._route = route
        self._navpts = self._build_nav_points(dets, pose)
        dt = 0.0 if self._last_update is None else clamp(now - self._last_update, 0.0, 0.5)
        self._last_update = now
        faults = faults or {}

        # ---- Lỗi hệ thống (ưu tiên sau dừng khẩn) ----
        if self.state != St.ESTOP:
            if faults:
                self._fault_msg = "; ".join(faults.values())
                if self.state != St.FAULT:
                    self._enter(St.FAULT)
            elif self.state == St.FAULT:
                self._info_msg = "Đã khắc phục lỗi – nhấn START để chạy tiếp"
                self._info_until = now + 8.0
                self._enter(St.IDLE)

        # ---- Vật cản: YOLO (người, vật) + camera thấy sàn bị chặn trong làn đường ----
        obst = self._obstacle_det(dets)
        ctx = self._prev_state if self.state == St.OBSTACLE else self.state
        theta = self._motion_theta
        plan_blocked = False
        if (self.state == St.OBSTACLE and ctx in (St.APPROACH, St.FOLLOW) and self._navpts is not None
                and self.cfg.NAV_AVOID_ENABLE):
            # đang dừng: còn khe nào đủ rộng để đi tiếp không? -> xét an toàn theo hướng khe đó
            res = self.planner.adjust(0.1, 0.0, 0.0, self._navpts, self._last_desired, self._last_look,
                                      self._free_stop_distance(ctx))
            plan_blocked = res.status == "blocked"
            if res.theta is not None:
                theta = res.theta
        lane_d = self.planner.lane_distance(self._navpts, theta)
        moving = self.state == St.OBSTACLE or self._last_trans > 0.02   # quay tại chỗ thì không xét làn
        floor_hit = bool(self.cfg.FLOOR_STOP_ENABLE and self._navpts is not None and moving
                         and ctx in (St.FOLLOW, St.APPROACH, St.MANUAL)
                         and (lane_d < self._free_stop_distance(ctx) or plan_blocked))
        if obst is not None or floor_hit:
            self._obst_frames += 1
            self._last_obst_t = now
        else:
            self._obst_frames = 0
        blocking = self._obst_frames >= self.cfg.OBST_ON_FRAMES
        check = self.state in (St.SEARCH, St.FOLLOW, St.APPROACH) and (
            self.state != St.SEARCH or self.cfg.OBST_CHECK_IN_SEARCH)
        if check and blocking:
            self._prev_state = self.state
            self._enter(St.OBSTACLE)

        out = MissionOutput(state=self.state, station=self.target, route=list(self.route),
                            route_idx=self.route_idx, lap=self.lap, obstacle_det=obst)
        dists = [lane_d] if math.isfinite(lane_d) else []
        if obst is not None:
            dists.append(self._obstacle_geom(obst)[0])
        out.obstacle_dist = min(dists) if dists else None
        handler = getattr(self, "_do_" + self.state.value.lower())
        handler(out, dets, yaw_deg, dt, blocking)
        out.state = self.state
        out.station = self.target
        out.route_idx = self.route_idx
        out.lap = self.lap
        if self._beep:
            out.beep, self._beep = self._beep, None
        # hướng chuyển động cho lần kiểm tra làn kế tiếp
        self._last_trans = math.hypot(out.vx, out.vy) if out.drive else 0.0
        if out.drive and out.vx > 0.02:
            self._motion_theta = math.atan2(out.vy, out.vx)
        elif out.drive and abs(out.vy) > 0.02:
            self._motion_theta = math.copysign(math.pi / 2, out.vy)
        elif not out.drive or out.state not in (St.OBSTACLE,):
            self._motion_theta = 0.0 if not out.drive else self._motion_theta
        if self.state == St.IDLE and self._info_msg and now < self._info_until:
            out.subtitle = self._info_msg
        return out

    # ------------------------------------------------------------------
    # Các trạng thái
    # ------------------------------------------------------------------
    @staticmethod
    def _halt(out: MissionOutput) -> None:
        """Trạng thái đứng yên: xóa mọi lệnh chuyển động."""
        out.vx = out.vy = out.wz = 0.0
        out.drive = False

    def _do_idle(self, out, dets, yaw, dt, blocking):
        self._halt(out)
        out.title = "SẴN SÀNG"
        out.subtitle = f"Trạm kế tiếp: {self.station_name()} – nhấn START để chạy tự động"
        out.level = "idle"

    def _do_done(self, out, dets, yaw, dt, blocking):
        self._halt(out)
        out.title = "HOÀN THÀNH LỘ TRÌNH"
        out.subtitle = "Nhấn START để chạy lại từ trạm đầu"
        out.level = "idle"

    def _do_estop(self, out, dets, yaw, dt, blocking):
        self._halt(out)
        out.title = "DỪNG KHẨN CẤP"
        src = f" ({self._estop_src})" if self._estop_src else ""
        out.subtitle = f"Nhả nút dừng khẩn{src}, kiểm tra an toàn rồi nhấn R để reset"
        out.level = "alarm"

    def _do_fault(self, out, dets, yaw, dt, blocking):
        self._halt(out)
        out.title = "LỖI HỆ THỐNG – ROBOT DỪNG"
        out.subtitle = self._fault_msg
        out.level = "alarm"

    def _do_lost(self, out, dets, yaw, dt, blocking):
        self._halt(out)
        c = self.cfg
        out.title = f"KHÔNG TÌM THẤY {self.station_name()}"
        if c.LOST_AUTO_RETRY_S > 0:
            remain = max(0.0, c.LOST_AUTO_RETRY_S - (self._now - self._t_state))
            out.subtitle = f"Kiểm tra biển báo / ánh sáng – tự tìm lại sau {math.ceil(remain)} s"
            if remain <= 0:
                self._enter(St.SEARCH)
        else:
            out.subtitle = "Kiểm tra biển báo / ánh sáng rồi nhấn START để tìm lại"
        out.level = "warn"

    def _do_search(self, out, dets, yaw, dt, blocking):
        c = self.cfg
        det = self._target_det(dets)
        out.title = f"ĐANG DI CHUYỂN ĐẾN {self.station_name()}"
        out.level = "move"
        out.target_det = det
        self._search_time += dt
        self._confirm = self._confirm + 1 if det is not None else 0
        if self._confirm >= c.CONFIRM_FRAMES:
            self._enter(St.APPROACH)
            self._do_approach(out, dets, yaw, 0.0, blocking)
            return
        if self._search_time > c.SEARCH_TIMEOUT_S:
            self._enter(St.LOST)
            self._do_lost(out, dets, yaw, dt, blocking)
            return

        direction = self._search_dir_override or int(self.station_cfg().get("search_dir", -1)) or -1
        wz = 0.0
        hint = self.station_cfg().get("heading_deg")
        if self._search_phase == "align" and (hint is None or yaw is None):
            self._search_phase = "sweep"
        if det is not None:
            wz = direction * c.W_SEARCH_SLOW
            out.subtitle = "Đã thấy biển báo – đang xác nhận..."
        elif self._search_phase == "align":
            err = wrap180(float(hint) - yaw)
            if abs(err) <= c.HEADING_TOL_DEG:
                self._search_phase = "sweep"
            wz = clamp(c.HEADING_KP * math.radians(err), -c.W_SEARCH, c.W_SEARCH)
            out.subtitle = f"Đang xoay về hướng trạm theo la bàn ({err:+.0f}°)"
        else:
            wz = direction * c.W_SEARCH
            out.subtitle = "Đang quay tìm biển báo trạm..."
        out.wz = wz
        out.drive = True

    def _do_approach(self, out, dets, yaw, dt, blocking):
        c = self.cfg
        det = self._target_det(dets)
        out.title = f"ĐANG DI CHUYỂN ĐẾN {self.station_name()}"
        out.level = "move"
        out.target_det = det
        out.drive = True
        if det is not None:
            a = clamp(c.TRACK_SMOOTH, 0.0, 0.95)
            if self._smooth is None:
                self._smooth = [det.cx, det.h]
            else:
                self._smooth[0] = a * self._smooth[0] + (1 - a) * det.cx
                self._smooth[1] = a * self._smooth[1] + (1 - a) * det.h
            cx, h = self._smooth
            self._last_seen = self._now
            ex = clamp((cx - 0.5) * 2.0, -1.0, 1.0)       # > 0: trạm lệch sang phải
            self._last_ex = ex
            marker_h = float(self.station_cfg().get("marker_h_m", 0.2))
            dist = self.f_norm * marker_h / max(h, 1e-3)
            out.distance = dist

            span = max(1e-3, c.SLOW_DISTANCE_M - c.ARRIVE_DISTANCE_M)
            f = clamp((dist - c.ARRIVE_DISTANCE_M) / span, 0.0, 1.0)
            v = c.V_APPROACH_MIN + (c.V_APPROACH_MAX - c.V_APPROACH_MIN) * f
            if dist <= c.ARRIVE_DISTANCE_M:
                v = 0.0
            g = clamp(1.0 - abs(ex) / max(1e-3, c.EX_TURN_ONLY), 0.0, 1.0)
            vx = v * g
            if c.APPROACH_MODE == "strafe":
                vy = -c.K_VY_CENTER * ex
                wz = -0.3 * c.K_W_CENTER * ex
            else:
                vy = 0.0
                wz = -c.K_W_CENTER * ex
            wz = clamp(wz, -c.W_MAX, c.W_MAX)

            self._station_dist = dist
            if dist <= c.ARRIVE_DISTANCE_M and abs(ex) <= c.ARRIVE_CENTER_TOL:
                self._arrive_cnt += 1
            else:
                self._arrive_cnt = 0
            if self._arrive_cnt >= c.ARRIVE_FRAMES:
                self._enter(St.WAIT)
                self._do_wait(out, dets, yaw, 0.0, blocking)
                return
            # bộ lái cục bộ: lách vật cản ở xa, chân biển trạm không tính (tầm nhìn < khoảng cách trạm)
            look = min(c.NAV_LOOKAHEAD_M, dist - 0.3)
            if look >= 0.4:
                vx, vy, wz, nav = self._plan(vx, vy, wz, self.geom.bearing(cx), look, St.APPROACH, dets)
            else:                                   # pha cuối sát trạm: chỉ còn dừng an toàn
                nav = "clear" if self._free is not None else "none"
            if nav == "blocked":
                self._to_obstacle(out, dets, yaw, blocking, St.APPROACH)
                return
            out.nav = nav
            self._last_cmd = (vx, vy, wz)
            out.vx, out.vy, out.wz = vx, vy, wz
            out.subtitle = f"Đã thấy biển báo – còn khoảng {dist:.1f} m" + (
                " – đang lách vật cản" if nav == "avoid" else "")
        else:
            lost = self._now - self._last_seen
            if lost < c.TARGET_LOST_HOLD_S:
                vx, vy, wz = self._last_cmd
                out.vx, out.vy, out.wz = 0.5 * vx, 0.5 * vy, 0.5 * wz
                out.subtitle = "Tạm mất dấu biển báo..."
            elif lost < c.TARGET_LOST_TIMEOUT_S:
                out.subtitle = "Mất dấu biển báo – đang chờ..."
            else:
                # quay về phía lần cuối nhìn thấy trạm
                self._search_dir_override = -1 if self._last_ex > 0 else 1
                self._enter(St.SEARCH)
                out.subtitle = "Mất dấu biển báo – tìm lại"

    def _to_obstacle(self, out, dets, yaw, blocking, prev: St) -> None:
        """Bộ lái báo không còn khe trống -> dừng, hiển thị GẶP VẬT CẢN."""
        self._prev_state = prev
        self._last_obst_t = self._now
        self._enter(St.OBSTACLE)
        self._do_obstacle(out, dets, yaw, 0.0, blocking)

    def _do_follow(self, out, dets, yaw, dt, blocking):
        c = self.cfg
        out.title = f"ĐANG DI CHUYỂN ĐẾN {self.station_name()}"
        out.level = "move"
        out.drive = True
        det = self._target_det(dets)
        out.target_det = det
        if det is not None and self.distance_from_det(det) <= c.ROUTE_HANDOFF_DISTANCE_M:
            self._confirm += 1
        else:
            self._confirm = 0
        if self._confirm >= c.CONFIRM_FRAMES:          # đã thấy biển trạm -> tiếp cận bằng YOLO
            self._enter(St.APPROACH)
            self._do_approach(out, dets, yaw, 0.0, blocking)
            return
        g = self._route
        if g is None:
            out.subtitle = "Đang nạp lộ trình đã dạy..."
            return
        out.route_kf = f"{g.idx + 1}/{g.n}"
        if g.done or g.lost:
            self._search_dir_override = None
            self._enter(St.SEARCH)
            out.subtitle = ("Hết lộ trình – tìm biển báo trạm" if g.done
                            else "Mất dấu lộ trình – quay tìm biển báo trạm")
            return
        if not g.ok:
            out.subtitle = f"Đang so khớp ảnh mốc... ({g.matches} điểm)"
            return
        vx, vy, wz, nav = self._plan(g.vx, 0.0, g.wz, g.desired, c.NAV_LOOKAHEAD_M, St.FOLLOW, dets)
        if nav == "blocked":
            self._to_obstacle(out, dets, yaw, blocking, St.FOLLOW)
            return
        out.nav = nav
        out.vx, out.vy, out.wz = vx, vy, wz
        out.subtitle = f"Theo lộ trình đã dạy – mốc {g.idx + 1}/{g.n} ({g.matches} điểm khớp)" + (
            " – lách vật cản" if nav == "avoid" else "")

    def _do_wait(self, out, dets, yaw, dt, blocking):
        self._halt(out)
        remain = self._wait_end - self._now
        if self._skip or remain <= 0:
            self._advance_route(yaw)
            if self.state == St.DONE:
                self._do_done(out, dets, yaw, dt, blocking)
            else:
                out.title = f"ĐANG DI CHUYỂN ĐẾN {self.station_name()}"
                out.subtitle = "Rời trạm..."
                out.level = "move"
            return
        name = self.station_name()
        out.title = f"ĐÃ ĐẾN {name}"
        out.subtitle = f"Mời bỏ rác / vải vụn vào thùng – còn {math.ceil(remain)} s"
        out.countdown = remain
        out.level = "wait"

    def _do_depart(self, out, dets, yaw, dt, blocking):
        c = self.cfg
        self._depart_time += dt
        out.title = f"ĐANG DI CHUYỂN ĐẾN {self.station_name()}"
        out.subtitle = "Đang lùi khỏi trạm"
        out.level = "move"
        if self._depart_time >= c.DEPART_REVERSE_S:
            self._start_leg()
            return
        out.vx = -abs(c.DEPART_REVERSE_V)
        if yaw is not None and self._hold_yaw is not None:
            out.wz = clamp(c.HEADING_KP * math.radians(wrap180(self._hold_yaw - yaw)), -c.W_MAX, c.W_MAX)
        out.drive = True

    def _do_obstacle(self, out, dets, yaw, dt, blocking):
        self._halt(out)
        c = self.cfg
        clear_for = self._now - self._last_obst_t
        if clear_for >= c.OBST_CLEAR_S:
            prev = self._prev_state or St.SEARCH
            self.state = prev
            self._t_state = self._now
            if prev == St.APPROACH:
                self._last_seen = self._now
                self._arrive_cnt = 0
            handler = getattr(self, "_do_" + prev.value.lower())
            handler(out, dets, yaw, 0.0, False)
            return
        out.title = "GẶP VẬT CẢN – ROBOT DỪNG"
        wait = max(0.0, c.OBST_CLEAR_S - clear_for)
        if blocking or clear_for < 0.05:
            od = out.obstacle_dist
            where = f"cách {od:.1f} m – " if od is not None and od < 5 else ""
            out.subtitle = f"Vật cản {where}đang chờ dọn đường... (đích: {self.station_name()})"
        else:
            out.subtitle = f"Đường đã thông – chạy tiếp sau {wait:.1f} s"
        out.level = "alarm"

    def _do_manual(self, out, dets, yaw, dt, blocking):
        c = self.cfg
        out.title = "ĐIỀU KHIỂN TAY"
        out.level = "manual"
        out.subtitle = "W/S: tiến/lùi  A/D: đi ngang  Q/E: quay  SPACE: dừng  M: thoát"
        if self._now - self._manual_t <= 0.6:
            vx, vy, wz = self._manual_cmd
        else:
            vx = vy = wz = 0.0
        if c.OBST_CHECK_IN_MANUAL and blocking and vx > 0:
            vx = 0.0
            out.subtitle = "CÓ VẬT CẢN PHÍA TRƯỚC – chặn lệnh tiến"
            out.level = "warn"
        out.vx, out.vy, out.wz = vx, vy, wz
        out.drive = True
