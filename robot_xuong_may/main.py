#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
 ROBOT THU GOM RÁC / VẢI VỤN XƯỞNG MAY – CHƯƠNG TRÌNH CHÍNH (Raspberry Pi 4)
=====================================================================
  Camera ─┬─> YOLO: biển TRẠM 1/2/3 + người / vật cản
         ├─> Nhận diện MẶT SÀN TRỐNG (gạch đá hoa / bê tông) -> khoảng trống từng hướng
         └─> So khớp ẢNH MỐC lộ trình đã dạy (teach & repeat)  -> dẫn đường, KHÔNG dán line
      -> Máy trạng thái + bộ lái cục bộ (lách / dừng) -> Động học mecanum
      -> Arduino UNO (SAC-DOB điều khiển 4 động cơ)
      -> Màn hình: "ĐANG DI CHUYỂN ĐẾN TRẠM x" / "ĐÃ ĐẾN TRẠM x" / "GẶP VẬT CẢN"

Chạy:
  python main.py                 # robot thật
  python main.py --auto-start    # tự chạy lộ trình ngay khi khởi động
  python main.py --sim           # mô phỏng toàn bộ trên laptop (không cần phần cứng)
  python main.py --fake-link     # camera + YOLO thật, Arduino giả (thử nhận diện)
  python main.py --fake-link --video thu.mp4      # chạy bằng video nhóm train gửi
  python main.py --fake-link --images anh_test/ --no-floor   # chạy bằng thư mục ảnh (trình chiếu)
  python main.py --port /dev/pts/5                # nối Arduino giả lập (tools/fake_arduino.py)
  python main.py --no-gui        # không màn hình (chạy qua SSH)

Phím (cửa sổ giao diện):
  ENTER/G chạy tự động | P dừng | M chế độ tay (W/S/A/D/Q/E, SPACE dừng)
  T dạy lộ trình (trong chế độ tay: nhấn T bắt đầu, lái tới trạm, nhấn T lưu) | U hủy dạy
  F học lại màu sàn (phía trước phải trống) | N bỏ qua thời gian chờ
  K dừng khẩn | R reset | C đổi bộ ĐK động cơ | ESC thoát
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot import fwconfig  # noqa: E402
from robot.display import Display  # noqa: E402
from robot.kinematics import MecanumKinematics  # noqa: E402
from robot.mission import Mission, St  # noqa: E402
from robot.odometry import Odometry  # noqa: E402
from robot.sac_dob import MODE_NAMES  # noqa: E402

log = logging.getLogger("main")


def setup_logging(verbose: bool) -> None:
    log_dir = PROJECT_DIR / cfg.LOG_DIR
    log_dir.mkdir(exist_ok=True)
    fmt = "%(asctime)s %(levelname)-7s %(name)-8s %(message)s"
    handlers = [logging.FileHandler(log_dir / time.strftime("robot_%Y%m%d.log"), encoding="utf-8"),
                logging.StreamHandler(sys.stdout)]
    handlers[1].setLevel(logging.DEBUG if verbose else logging.WARNING)
    logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO, format=fmt, handlers=handlers)


def parse_args():
    ap = argparse.ArgumentParser(description="Robot thu gom rác xưởng may")
    ap.add_argument("--sim", action="store_true", help="mô phỏng toàn bộ (không cần phần cứng)")
    ap.add_argument("--fake-link", action="store_true", help="dùng Arduino giả lập")
    ap.add_argument("--no-gui", action="store_true", help="không mở cửa sổ giao diện")
    ap.add_argument("--fullscreen", action="store_true", help="giao diện toàn màn hình")
    ap.add_argument("--auto-start", action="store_true", help="tự chạy lộ trình khi khởi động")
    ap.add_argument("--port", default=None, help="cổng serial Arduino (mặc định theo config)")
    ap.add_argument("--model", default=None, help="đường dẫn mô hình YOLO")
    ap.add_argument("--video", default=None, help="dùng file video thay camera (thử nghiệm)")
    ap.add_argument("--images", default=None, help="dùng thư mục ảnh thay camera (thử nghiệm)")
    ap.add_argument("--camera", type=int, default=None,
                    help="số webcam USB (0, 1, 2...) thay CAMERA_INDEX – xem: python examples/01_xem_camera.py --list")
    ap.add_argument("--fps", type=float, default=2.0, help="tốc độ trình chiếu khi dùng --images")
    ap.add_argument("--no-compass", action="store_true", help="không dùng góc quay (con quay MPU-6050 / la bàn)")
    ap.add_argument("--no-floor", action="store_true",
                    help="tắt nhận diện sàn (khi thử bằng ảnh/video KHÔNG quay từ camera robot)")
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap.parse_args()


class RobotApp:
    def __init__(self, args):
        self.args = args
        self.kin = MecanumKinematics.from_config(cfg)
        self.odo = Odometry(self.kin)
        self.world = None
        self.floor = None
        self.routes = None
        self._teach_leg = None
        self._learn_floor = False
        self._info_msg = ("", 0.0)
        self.model_ok = True                       # mô hình có đủ / đúng thứ tự lớp trạm
        self.gpio = None
        self.compass = None
        self.running = True
        self.warnings = []
        self._batt_low_since = None
        self._batt_fault = False
        self._last_state = None
        self._fps = 0.0
        self._last_loop = time.monotonic()
        self._last_world_step = time.monotonic()
        self._t_start = time.monotonic()
        self.expected_hash = fwconfig.config_hash(cfg)
        self._build()

    # ----------------------------------------------------------------
    def _build(self) -> None:
        a = self.args
        if a.sim:
            from robot.simworld import SimCamera, SimCompass, SimDetector, SimLink, SimWorld
            self.link = SimLink(cfg, config_hash=self.expected_hash).start_thread()
            self.world = SimWorld(cfg, self.kin)
            self.camera = SimCamera(self.world, cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT).start()
            self.detector = SimDetector(self.world)
            if not cfg.IMU_ENABLE:                        # có MPU-6050: góc quay lấy từ SimLink (Arduino giả)
                self.compass = SimCompass(self.world)
            from robot.simworld import SimRouteManager
            self.routes = SimRouteManager(cfg, self.world)
            log.info("CHẾ ĐỘ MÔ PHỎNG – phím O: đặt/xóa người chắn đường trước robot")
        else:
            # --- Arduino ---
            if a.fake_link:
                from robot.simworld import SimLink
                self.link = SimLink(cfg, config_hash=self.expected_hash).start_thread()
            else:
                from robot.serial_link import PortError, SerialLink
                try:
                    self.link = SerialLink(a.port or cfg.SERIAL_PORT, cfg.SERIAL_BAUD).open()
                except PortError as exc:
                    raise SystemExit(f"{exc}\n(Thử không cần Arduino: python main.py --sim)")
            # --- Camera ---
            from robot.camera import Camera
            backend = "opencv" if a.camera is not None else cfg.CAMERA_BACKEND
            index = cfg.CAMERA_INDEX if a.camera is None else a.camera
            try:
                self.camera = Camera(backend, index, cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT,
                                     cfg.CAMERA_FLIP, video_path=a.images or a.video, image_fps=a.fps).start()
            except RuntimeError as exc:
                self.link.close()
                raise SystemExit(f"{exc}\n  -> Xem các webcam đang có: python examples/01_xem_camera.py --list  (rồi dùng --camera <số>)")
            frame = self._wait_first_frame()
            if frame is None:
                log.error("Camera chưa có hình ảnh sau 5 s")
            # --- YOLO ---
            from robot.detector import NullDetector, check_classes, create_detector
            try:
                self.detector = create_detector(a.model or cfg.MODEL_PATH, cfg.YOLO_IMGSZ, cfg.YOLO_CONF,
                                                cfg.YOLO_IOU)
                errors, notes = check_classes(self.detector.names, cfg)
                for e in errors:                      # sai tên / thứ tự lớp -> không cho chạy tự động
                    log.error("Kiểm tra lớp: %s", e)
                    self.warnings.append(e)
                for n_ in notes:
                    log.warning("Kiểm tra lớp: %s", n_)
                self.model_ok = not errors
                if frame is not None:
                    self.detector.warmup(frame)
            except Exception as exc:
                log.error("YOLO: %s", exc)
                self.warnings.append("Chưa có mô hình YOLO – chỉ chạy được chế độ tay")
                self.detector = NullDetector()
                self.model_ok = False
            # --- Góc quay: con quay MPU-6050 trên Arduino (gửi kèm telemetry) hoặc la bàn từ trên Pi ---
            if cfg.IMU_ENABLE:
                who = (getattr(self.link, "info", None) or {}).get("imu", 0)
                if who:
                    log.info("Con quay MPU-6050 trên Arduino: WHO_AM_I = 0x%02X", who)
                else:
                    log.warning("Arduino chưa thấy MPU-6050 (SDA->A4, SCL->A5, VCC->5V, GND) – robot vẫn chạy, "
                                "không giữ hướng được")
            elif cfg.COMPASS_ENABLE and not a.no_compass:
                try:
                    from robot.compass import Compass
                    self.compass = Compass(cfg.COMPASS_TYPE, cfg.COMPASS_I2C_BUS, cfg.COMPASS_CAL_FILE).start()
                except Exception as exc:
                    log.warning("Không dùng được la bàn: %s", exc)
                    self.compass = None
            # --- Dẫn đường bằng camera: sàn trống + lộ trình đã dạy ---
            if cfg.FLOOR_ENABLE and not a.no_floor:
                from robot.floor import FloorDetector
                self.floor = FloorDetector(cfg)
            if cfg.ROUTE_ENABLE:
                try:
                    from robot.route_memory import RouteManager
                    self.routes = RouteManager(cfg)
                    log.info("Lộ trình đã dạy: %s", self.routes.legs() or "chưa có")
                except Exception as exc:
                    log.warning("Không dùng được dạy-lặp lộ trình: %s", exc)
            # --- Nút nhấn / còi ---
            if cfg.GPIO_ENABLE:
                try:
                    from robot.gpio_io import GpioPanel
                    self.gpio = GpioPanel(cfg.BTN_START_PIN, cfg.BTN_STOP_PIN, cfg.BUZZER_PIN)
                except Exception as exc:
                    log.warning("Không dùng được GPIO: %s", exc)
        self.mission = Mission(cfg, routes=self.routes)
        self._check_firmware()
        self.display = Display(cfg, gui=not a.no_gui and cfg.DISPLAY_GUI,
                               fullscreen=a.fullscreen or cfg.DISPLAY_FULLSCREEN, lcd=cfg.LCD1602_ENABLE)
        self.link.set_controller(cfg.CONTROLLER_MODE)
        if a.auto_start and self.model_ok:
            self.mission.start()

    def _wait_first_frame(self, timeout: float = 5.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            frame, _ = self.camera.read()
            if frame is not None:
                return frame
            time.sleep(0.05)
        return None

    def _check_firmware(self) -> None:
        info = getattr(self.link, "info", None)
        if not info:
            self.warnings.append("Arduino chưa báo thông tin firmware")
        elif info.get("hash") not in (self.expected_hash, None):
            msg = "Firmware Arduino khác config.py – chạy: python flash_arduino.py"
            log.warning("%s (Arduino=%s, config=%s)", msg, info.get("hash"), self.expected_hash)
            self.warnings.append(msg)

    # ----------------------------------------------------------------
    def _faults(self, now: float, cam_ok: bool) -> dict:
        faults = {}
        if not self.link.alive(cfg.LINK_TIMEOUT_S):
            faults["link"] = "Mất kết nối Arduino (kiểm tra cáp USB)"
        if not cam_ok:
            faults["camera"] = "Lỗi camera – không có hình ảnh"
        vb = self.link.telemetry.vbat
        if cfg.BATTERY_SENSE_ENABLE and vb > 5.0:                 # > 5 V: có đo được pin
            if vb < cfg.BATTERY_CRIT_V:
                if self._batt_low_since is None:
                    self._batt_low_since = now
                if now - self._batt_low_since > 3.0:
                    self._batt_fault = True
            else:
                self._batt_low_since = None
                if vb > cfg.BATTERY_CRIT_V + 0.4:
                    self._batt_fault = False
            if self._batt_fault:
                faults["battery"] = f"Pin yếu {vb:.1f} V – hãy sạc pin 3S"
        return faults

    def _handle_key(self, k: int, now: float) -> None:
        if k < 0:
            return
        m = self.mission
        ch = chr(k).lower() if 0 < k < 128 else ""
        if k == 27:                                  # ESC
            self.running = False
        elif k in (13, 10) or ch == "g":
            if not self.model_ok:
                self._flash("Mô hình YOLO thiếu / sai lớp trạm – không chạy tự động được (xem log)")
            else:
                m.start()
        elif ch == "p":
            m.pause()
        elif ch == "m":
            if self.routes is not None and self.routes.teaching:
                self._stop_teach(save=False)
            m.toggle_manual()
        elif ch == "t":
            self._toggle_teach()
        elif ch == "u" and self.routes is not None and self.routes.teaching:
            self._stop_teach(save=False)
            self._flash("Đã hủy dạy lộ trình")
        elif ch == "f":
            if self.floor is not None:
                self.floor.learn_now()
                self._flash("Đã học lại màu sàn")
        elif ch == "n":
            m.skip_wait()
        elif ch == "k":
            self.link.estop(True)
            m.trigger_estop("Phần mềm")
        elif ch == "r":
            self.link.estop(False)
            m.reset()
        elif ch == "c":
            mode = (self.link.telemetry.mode + 1) % 3
            self.link.set_controller(mode)
            log.info("Đổi bộ điều khiển động cơ -> %s", MODE_NAMES[mode])
        elif ch == "o" and self.world is not None:
            self.world.toggle_obstacle_ahead()
        elif m.state == St.MANUAL:
            v, w = cfg.MANUAL_V, cfg.MANUAL_W
            cmd = {"w": (v, 0, 0), "s": (-v, 0, 0), "a": (0, v, 0), "d": (0, -v, 0),
                   "q": (0, 0, w), "e": (0, 0, -w), " ": (0, 0, 0)}.get(ch)
            if cmd is not None:
                m.set_manual(*cmd, now)

    def _flash(self, msg: str, seconds: float = 4.0) -> None:
        self._info_msg = (msg, time.monotonic() + seconds)
        log.info(msg)

    def _toggle_teach(self) -> None:
        """T: bắt đầu / kết thúc dạy chặng (trạm vừa rời -> trạm đích hiện tại)."""
        if self.routes is None:
            self._flash("Chức năng dạy lộ trình đang tắt (ROUTE_ENABLE)")
            return
        if self.routes.teaching:
            self._stop_teach(save=True)
            return
        if self.mission.state != St.MANUAL:
            self.mission.toggle_manual()
        if self.mission.state != St.MANUAL:
            return
        frm, to = self.mission.prev_station, self.mission.target
        self.routes.start_teach(frm, to)
        self._teach_leg = (frm, to)

    def _stop_teach(self, save: bool) -> None:
        mem = self.routes.stop_teach(save, self.odo.dist, self.odo.heading_deg)
        leg, self._teach_leg = self._teach_leg, None
        if save and mem is not None and leg is not None:
            self.mission.mark_at_station(leg[1])
            self._flash(f"Đã lưu lộ trình {leg[0]} → {leg[1]}. Trạm kế tiếp: {self.mission.station_name()}")
        elif save:
            self._flash("Lộ trình quá ngắn / thiếu đặc trưng – chưa lưu")

    # ----------------------------------------------------------------
    def step(self) -> None:
        now = time.monotonic()
        dt_loop = now - self._last_loop
        self._last_loop = now
        if dt_loop > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt_loop) if self._fps else 1.0 / dt_loop

        if self.world is not None:                    # mô phỏng: tích phân vị trí robot
            dt = now - self._last_world_step
            self._last_world_step = now
            self.world.step(min(dt, 0.2), self.link.telemetry.rpm)

        frame, t_frame = self.camera.read()
        cam_ok = frame is not None and (now - t_frame) < cfg.CAMERA_TIMEOUT_S
        dets = self.detector.detect(frame) if cam_ok else []

        tel = self.link.telemetry
        yaw = self._yaw(tel, now)
        self.odo.update(tel.rpm, now, yaw)
        pose = (self.odo.x, self.odo.y, self.odo.th)

        # ---- dẫn đường bằng camera ----
        free = None
        if self.world is not None:
            free = self.world.free_space()
        elif self.floor is not None and cam_ok:
            free = self.floor.process(frame, learn=self._learn_floor)
        route = None
        if self.routes is not None:
            if self.mission.consume_route_reset():
                self.routes.start_leg(self.mission.prev_station, self.mission.target, frame, self.odo.dist)
            if self.mission.wants_route and cam_ok:
                route = self.routes.follow(frame, self.odo.dist, now)
            if self.routes.teaching and cam_ok:
                self.routes.teach_feed(frame, self.odo.dist, self.odo.heading_deg)
        if tel.estop and self.mission.state != St.ESTOP:
            self.mission.trigger_estop("Nút dừng khẩn / Arduino")

        if self.gpio is not None:
            if self.gpio.take_start():
                if self.mission.state in (St.ESTOP, St.FAULT):
                    self.link.estop(False)
                    self.mission.reset()
                else:
                    self.mission.start()
            if self.gpio.take_stop():
                self.mission.pause()

        out = self.mission.update(now, dets, yaw, self._faults(now, cam_ok), free=free, route=route, pose=pose)
        # chỉ học màu sàn khi đang TIẾN trên đường trống (vùng tham chiếu chắc chắn là sàn)
        self._learn_floor = bool(out.drive and out.vx > 0.05 and out.state in (St.FOLLOW, St.APPROACH, St.MANUAL)
                                 and (out.obstacle_dist is None or out.obstacle_dist > 1.3))
        if self.routes is not None and self.routes.teaching and self._teach_leg:
            n, dist = self.routes.teach_info()
            a, b = self._teach_leg
            out.title = f"ĐANG DẠY LỘ TRÌNH: {'XUẤT PHÁT' if a == 0 else f'TRẠM {a}'} → TRẠM {b}"
            out.subtitle = f"Lái tay tới trước biển trạm rồi nhấn T để lưu ({n} ảnh mốc, {dist:.1f} m) – U: hủy"
            out.level = "manual"
        msg, until = self._info_msg
        if msg and now < until:
            out.subtitle = msg

        # ---- gửi lệnh xuống Arduino (gửi đều đặn = tín hiệu "còn sống") ----
        if out.drive:
            self.link.send_wheels(self.kin.inverse(out.vx, out.vy, out.wz))
        else:
            self.link.stop()

        if out.beep and self.gpio is not None:
            self.gpio.beep(out.beep)
        if out.state != self._last_state:
            log.info("Trạng thái %s -> %s | %s | %s", self._last_state, out.state.value, out.title, out.subtitle)
            self._last_state = out.state

        warnings = list(self.warnings)
        if cfg.IMU_ENABLE and yaw is None and now - self._t_start > 4.0:
            warnings.append("Không đọc được con quay MPU-6050 (A4/A5) – không giữ hướng")
        if cfg.BATTERY_SENSE_ENABLE and 5.0 < tel.vbat < cfg.BATTERY_WARN_V:
            warnings.append(f"Pin yếu: {tel.vbat:.1f} V")
        info = {"vbat": tel.vbat if tel.vbat > 5.0 else None, "yaw": yaw, "rpm": tel.rpm,
                "yaw_label": "Góc quay (MPU-6050)" if cfg.IMU_ENABLE else "Hướng la bàn",
                "ctrl": MODE_NAMES.get(tel.mode, "?"), "fps": self._fps,
                "yolo_ms": getattr(self.detector, "last_ms", 0.0), "warnings": warnings,
                "model": f"{getattr(self.detector, 'version', '?')} ({getattr(self.detector, 'backend', '?')})",
                "free": free, "floor_enabled": self.floor is not None or self.world is not None,
                "legs": self.routes.legs() if self.routes is not None else None}
        self.display.render(frame if cam_ok else None, dets, out, info)
        self._handle_key(self.display.poll_key(1), now)

        spent = time.monotonic() - now
        if spent < cfg.LOOP_MIN_PERIOD_S:
            time.sleep(cfg.LOOP_MIN_PERIOD_S - spent)

    def _yaw(self, tel, now: float):
        """Góc quay robot (độ, dương = quay trái) hoặc None nếu không có cảm biến / dữ liệu cũ."""
        if cfg.IMU_ENABLE and not self.args.no_compass:
            fresh = (now - getattr(tel, "rx_time", 0.0)) < 1.0
            return tel.yaw if (fresh and getattr(tel, "yaw_ok", False)) else None
        return self.compass.yaw_deg if self.compass is not None else None

    def run(self) -> None:
        log.info("Robot sẵn sàng. Lộ trình: %s", cfg.ROUTE)
        try:
            while self.running:
                self.step()
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        log.info("Dừng chương trình – dừng động cơ")
        for fn in (self.link.stop, self.link.close, self.camera.stop, self.display.close):
            try:
                fn()
            except Exception:
                pass
        if self.compass is not None and hasattr(self.compass, "stop"):
            self.compass.stop()
        if self.gpio is not None:
            self.gpio.close()


def main() -> int:
    args = parse_args()
    setup_logging(args.verbose)
    app = RobotApp(args)

    def on_signal(signum, frame):
        app.running = False
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
