# -*- coding: utf-8 -*-
"""
MÔ PHỎNG (chạy thử trên laptop khi chưa có phần cứng):  python main.py --sim

  * SimLink  : thay Arduino – bánh xe bám tốc độ đặt theo khâu quán tính bậc 1
  * SimWorld : xưởng 2D có 3 biển báo trạm + vật cản; sinh Detection giống YOLO
               từ hình học camera, vẽ khung hình giả để giao diện hiển thị.
"""
from __future__ import annotations

import math
import random
import threading
import time
from typing import List, Optional, Sequence, Tuple

from .common import Detection, clamp, focal_norm
from .serial_link import Telemetry


class SimLink:
    """Arduino giả lập – cùng API với SerialLink."""

    def __init__(self, cfg, tau: float = 0.12, config_hash: str = "sim"):
        self.cfg = cfg
        self.tau = tau
        self.telemetry = Telemetry(vbat=12.4, mode=cfg.CONTROLLER_MODE)
        self.info = {"name": "SIM", "version": "sim", "hash": config_hash,
                     "ts_ms": cfg.CONTROL_PERIOD_MS, "mode": cfg.CONTROLLER_MODE}
        self.params = {}
        self.port = "SIM"
        self._target = [0.0] * 4
        self._rpm = [0.0] * 4
        self._estop = False
        self._last_cmd = -1e9
        self._clock = 0.0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def open(self, *a, **k) -> "SimLink":
        return self

    def start_thread(self) -> "SimLink":
        def run():
            last = time.monotonic()
            while not self._stop.is_set():
                time.sleep(0.02)
                now = time.monotonic()
                self.step(now - last, now)
                last = now
        self._thread = threading.Thread(target=run, name="sim-link", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()

    def step(self, dt: float, now: Optional[float] = None) -> None:
        """Cập nhật mô hình bánh xe dt giây."""
        self._clock = now if now is not None else self._clock + dt
        with self._lock:
            if self._clock - self._last_cmd > self.cfg.CMD_TIMEOUT_MS / 1000.0 or self._estop:
                self._target = [0.0] * 4
            a = clamp(dt / self.tau, 0.0, 1.0)
            for i in range(4):
                self._rpm[i] += a * (self._target[i] - self._rpm[i])
            flags = (1 if self._estop else 0) | (2 if self._clock - self._last_cmd > self.cfg.CMD_TIMEOUT_MS / 1000.0 else 0)
            vbat = max(10.9, self.telemetry.vbat - 1e-5 * dt)
            self.telemetry = Telemetry(t_ms=int(self._clock * 1000), rpm=list(self._rpm),
                                       u=[x / self.cfg.MOTOR_RPM_NOMINAL for x in self._rpm],
                                       flags=flags, mode=self.telemetry.mode, vbat=vbat,
                                       rx_time=time.monotonic())

    # ---- API giống SerialLink ----
    def alive(self, timeout: float = 1.0) -> bool:
        return True

    def send_wheels(self, rpm: Sequence[float]) -> None:
        with self._lock:
            self._last_cmd = self._clock
            if not self._estop:
                self._target = [clamp(x, -self.cfg.MOTOR_RPM_NOMINAL, self.cfg.MOTOR_RPM_NOMINAL) for x in rpm]

    def send_open_loop(self, duty: Sequence[float]) -> None:
        self.send_wheels([d * self.cfg.MOTOR_RPM_NOMINAL for d in duty])

    def stop(self) -> None:
        with self._lock:
            self._last_cmd = self._clock
            self._target = [0.0] * 4

    def estop(self, active: bool = True) -> None:
        self._estop = bool(active)

    def set_controller(self, mode: int) -> None:
        self.telemetry.mode = int(mode)

    def reset_gains(self) -> None:
        pass

    def set_param(self, name: str, value: float) -> None:
        self.params[name] = value

    def query_params(self, wait_s: float = 1.0):
        return dict(self.params)

    def debug_motor(self, idx: int) -> None:
        pass

    def request_info(self) -> None:
        pass


class SimWorld:
    """Xưởng may 2D. Đơn vị: m, rad. Robot bắt đầu ở (0, 0) nhìn theo trục +x."""

    DEFAULT_STATIONS = {1: (3.0, -0.6), 2: (3.2, 3.0), 3: (-0.4, 3.2)}

    def __init__(self, cfg, kinematics, stations=None, seed: int = 1, flicker: float = 0.05,
                 station_range: float = 7.0, station_base_r: float = 0.15):
        self.cfg = cfg
        self.kin = kinematics
        self.rng = random.Random(seed)
        self.flicker = flicker
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.stations = dict(stations or self.DEFAULT_STATIONS)
        # vật cản: (x, y, bán kính, chiều cao, lớp)
        self.obstacles: List[Tuple[float, float, float, float, str]] = []
        self.fy = focal_norm(cfg.CAMERA_VFOV_DEG)
        self.fx = focal_norm(cfg.CAMERA_HFOV_DEG)
        self.hfov = math.radians(cfg.CAMERA_HFOV_DEG)
        self.cam_h = cfg.CAMERA_HEIGHT_M
        self.max_range = 7.0
        self.station_range = station_range      # nhỏ -> giả lập trạm bị khuất (cần lộ trình đã dạy)
        self.station_base_r = station_base_r    # chân biển / bàn / thùng của trạm (vật cản thật)
        from .geometry import CameraGeometry
        self.geom = CameraGeometry.from_config(cfg)

    # ---------------------------------------------------------------
    @property
    def yaw_deg(self) -> float:
        """La bàn giả lập (CCW dương)."""
        return math.degrees(self.th) % 360.0

    def step(self, dt: float, wheel_rpm: Sequence[float]) -> None:
        vx, vy, wz = self.kin.forward(wheel_rpm)
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.th = math.atan2(math.sin(self.th + wz * dt), math.cos(self.th + wz * dt))

    def toggle_obstacle_ahead(self, dist: float = 1.2) -> None:
        if self.obstacles:
            self.obstacles.clear()
        else:
            self.obstacles.append((self.x + dist * math.cos(self.th), self.y + dist * math.sin(self.th),
                                   0.25, 1.6, "nguoi"))

    def _project(self, px: float, py: float) -> Tuple[float, float]:
        """-> (khoảng cách, góc phương vị so với hướng camera; dương = bên trái)."""
        dx, dy = px - self.x, py - self.y
        return math.hypot(dx, dy), math.atan2(math.sin(math.atan2(dy, dx) - self.th),
                                              math.cos(math.atan2(dy, dx) - self.th))

    def detections(self) -> List[Detection]:
        dets: List[Detection] = []
        for sid, (px, py) in self.stations.items():
            st = self.cfg.STATIONS[sid]
            d, b = self._project(px, py)
            if d < 0.15 or d > min(self.max_range, self.station_range) or abs(b) > self.hfov / 2:
                continue
            if self.rng.random() < self.flicker:
                continue                                    # mô phỏng nhận diện chập chờn
            h = self.fy * st["marker_h_m"] / d
            w = self.fx * st["marker_h_m"] / d
            cx = 0.5 - (math.tan(b) * self.fx)
            dets.append(Detection(st["class"], 0.85 + 0.1 * self.rng.random(),
                                  clamp(cx - w / 2, 0, 1), clamp(0.5 - h / 2, 0, 1),
                                  clamp(cx + w / 2, 0, 1), clamp(0.5 + h / 2, 0, 1)))
        for (ox, oy, r, oh, cls) in self.obstacles:
            d, b = self._project(ox, oy)
            near = max(0.1, d - r)
            if d > self.max_range or abs(b) > self.hfov / 2 + math.atan2(r, max(d, 1e-3)):
                continue
            w = self.fx * 2 * r / near
            cx = 0.5 - (math.tan(b) * self.fx)
            y2 = 0.5 + self.fy * self.cam_h / near
            y1 = 0.5 - self.fy * (oh - self.cam_h) / near
            dets.append(Detection(cls, 0.9, clamp(cx - w / 2, 0, 1), clamp(y1, 0, 1),
                                  clamp(cx + w / 2, 0, 1), clamp(y2, 0, 1)))
        return dets

    def add_box(self, x: float, y: float, r: float = 0.2, h: float = 0.5, cls: str = "vat_can") -> None:
        self.obstacles.append((x, y, r, h, cls))

    def _circles(self):
        out = [(ox, oy, r) for (ox, oy, r, _, _) in self.obstacles]
        out += [(px, py, self.station_base_r) for (px, py) in self.stations.values()]
        return out

    def free_space(self):
        """Cảm biến "sàn trống" giả lập: mỗi tia camera -> khoảng cách tiến tới vật gần nhất."""
        import numpy as np
        from .floor import FreeSpace
        nb = int(self.cfg.FLOOR_BINS)
        rmax = float(self.cfg.FLOOR_MAX_RANGE_M)
        xs = (np.arange(nb) + 0.5) / nb
        dist = np.full(nb, rmax)
        lat = np.zeros(nb)
        rows = np.full(nb, np.nan)
        circles = self._circles()
        x_min = self.geom.ground_distance(1.0)
        for k, x in enumerate(xs):
            b = self.geom.bearing(float(x))
            ang = self.th + b
            ux, uy = math.cos(ang), math.sin(ang)
            best = None
            for (cx, cy, r) in circles:
                dx, dy = cx - self.x, cy - self.y
                t = dx * ux + dy * uy
                q = dx * dx + dy * dy - t * t
                if t > 0 and q < r * r:
                    hit = t - math.sqrt(r * r - q)
                    if hit > 0 and (best is None or hit < best):
                        best = hit
            if best is not None:
                # camera thật không thấy sàn gần hơn đáy khung hình -> vật quá gần báo ở khoảng đó
                X = max(best * math.cos(b), x_min)
                if X < rmax:
                    dist[k] = X
                    lat[k] = X * math.tan(b)
                    rows[k] = min(1.0, self.geom.row_of_distance(X))
        return FreeSpace(valid=True, dist=dist, lat=lat, xs=xs, rows=rows, max_range=rmax, learned=True)

    def clearance(self) -> float:
        """Khoảng cách nhỏ nhất từ thân robot (tròn bán kính ROBOT_HALF_WIDTH_M) tới vật cản."""
        r_robot = self.cfg.ROBOT_HALF_WIDTH_M
        return min((math.hypot(ox - self.x, oy - self.y) - r - r_robot
                    for (ox, oy, r, _, _) in self.obstacles), default=float("inf"))

    def render(self, width: int, height: int):
        """Khung hình giả (BGR) để giao diện có hình ảnh khi mô phỏng."""
        import numpy as np
        import cv2
        img = np.zeros((height, width, 3), np.uint8)
        img[: height // 2] = (70, 60, 50)              # tường
        for yy in range(height // 2, height):           # sàn có độ dốc màu
            v = int(90 + 80 * (yy - height / 2) / (height / 2))
            img[yy] = (v, v, v - 10)
        colors = {1: (200, 120, 30), 2: (40, 170, 40), 3: (40, 40, 200)}
        for det in self.detections():
            x1, y1, x2, y2 = int(det.x1 * width), int(det.y1 * height), int(det.x2 * width), int(det.y2 * height)
            if det.cls in self.cfg.OBSTACLE_CLASSES:
                cv2.rectangle(img, (x1, y1), (x2, y2), (40, 40, 40), -1)
                continue
            sid = next((k for k, v in self.cfg.STATIONS.items() if v["class"] == det.cls), 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), colors.get(sid, (180, 180, 180)), -1)
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 2)
            fs = max(0.4, (y2 - y1) / 40.0)
            cv2.putText(img, str(sid), (x1 + (x2 - x1) // 4, y2 - (y2 - y1) // 5),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), max(1, int(fs * 2)))
        return img

    def distance_to_station(self, sid: int) -> float:
        px, py = self.stations[sid]
        return math.hypot(px - self.x, py - self.y)


class SimCamera:
    """Camera giả: trả về khung hình vẽ từ SimWorld."""

    def __init__(self, world: SimWorld, width: int, height: int):
        self.world = world
        self.width = width
        self.height = height
        self.active_backend = "sim"
        self.frame_count = 0

    def start(self) -> "SimCamera":
        return self

    def read(self):
        self.frame_count += 1
        return self.world.render(self.width, self.height), time.monotonic()

    def stop(self) -> None:
        pass


class SimDetector:
    """YOLO giả: lấy Detection trực tiếp từ hình học SimWorld."""
    names = {}

    def __init__(self, world: SimWorld):
        self.world = world
        self.last_ms = 0.0

    def warmup(self, frame) -> None:
        pass

    def detect(self, frame):
        return self.world.detections()


class SimCompass:
    def __init__(self, world: SimWorld):
        self.world = world

    @property
    def yaw_deg(self):
        return self.world.yaw_deg

    def stop(self) -> None:
        pass


class SimRouteManager:
    """Dạy – lặp lộ trình trong mô phỏng: ghi tư thế thật của robot khi lái tay,
    khi lặp thì bám theo các điểm đã ghi (pure pursuit). Cùng API với RouteManager."""

    def __init__(self, cfg, world: SimWorld):
        self.cfg = cfg
        self.world = world
        self.legs_data = {}
        self._rec = None
        self._leg = None
        self._t = 0
        self.leg = None
        self.half_fov = math.radians(cfg.CAMERA_HFOV_DEG) / 2.0

    def has(self, frm, to):
        return (int(frm), int(to)) in self.legs_data

    def legs(self):
        return sorted(self.legs_data)

    def set_leg(self, frm, to, points):
        self.legs_data[(int(frm), int(to))] = list(points)

    @property
    def teaching(self):
        return self._rec is not None

    def start_teach(self, frm, to):
        self._rec = (int(frm), int(to), [(self.world.x, self.world.y)])

    def teach_feed(self, frame, odo_dist, heading_deg):
        if self._rec is None:
            return
        pts = self._rec[2]
        if math.hypot(self.world.x - pts[-1][0], self.world.y - pts[-1][1]) >= 0.2:
            pts.append((self.world.x, self.world.y))

    def teach_info(self):
        if self._rec is None:
            return 0, 0.0
        pts = self._rec[2]
        return len(pts), sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))

    def stop_teach(self, save, odo_dist=0.0, heading_deg=0.0):
        rec, self._rec = self._rec, None
        if rec is None or not save or len(rec[2]) < 2:
            return None
        rec[2].append((self.world.x, self.world.y))
        self.legs_data[(rec[0], rec[1])] = rec[2]
        return rec

    def start_leg(self, frm, to, frame, odo_dist):
        self._leg = self.legs_data.get((int(frm), int(to)))
        self.leg = (int(frm), int(to)) if self._leg else None
        if self._leg:
            d = [math.hypot(p[0] - self.world.x, p[1] - self.world.y) for p in self._leg]
            self._t = int(min(range(len(d)), key=d.__getitem__))
        return self._leg is not None

    def follow(self, frame, odo_dist, now=None):
        from .route_memory import RouteGuidance
        pts = self._leg
        if not pts:
            return None
        w = self.world
        while self._t < len(pts) - 1 and math.hypot(pts[self._t][0] - w.x, pts[self._t][1] - w.y) < 0.35:
            self._t += 1
        px, py = pts[self._t]
        b = math.atan2(py - w.y, px - w.x) - w.th
        b = math.atan2(math.sin(b), math.cos(b))
        ex = clamp(-b / self.half_fov, -1.0, 1.0)
        c = self.cfg
        rem = math.hypot(pts[-1][0] - w.x, pts[-1][1] - w.y)
        g = RouteGuidance(ok=True, idx=self._t, n=len(pts), matches=120, ex=ex,
                          wz=clamp(-c.ROUTE_K_W * ex, -c.W_MAX, c.W_MAX), desired=clamp(b, -self.half_fov, self.half_fov),
                          remaining=rem)
        g.vx = c.ROUTE_V * clamp(1.0 - abs(ex) / max(1e-3, c.EX_TURN_ONLY), 0.0, 1.0)
        g.done = self._t == len(pts) - 1 and rem < 0.3
        return g

    def stop_leg(self):
        self._leg = None
        self.leg = None
