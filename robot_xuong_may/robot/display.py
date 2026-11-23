# -*- coding: utf-8 -*-
"""
MÀN HÌNH VẬN HÀNH (HMI) trên Raspberry Pi
  * GUI: cửa sổ OpenCV 800x480 (màn HDMI/DSI 5"-7"), chữ tiếng Việt có dấu (vẽ bằng Pillow)
      ┌───────────────────────────────────────────────┐
      │  ĐANG DI CHUYỂN ĐẾN TRẠM 2      (băng trạng thái màu)
      │  Đã thấy biển báo – còn khoảng 1.2 m          │
      ├──────────────────────────┬────────────────────┤
      │                          │ LỘ TRÌNH ① ❷ ③     │
      │   ảnh camera + khung     │ đếm ngược / thông số│
      │   nhận diện YOLO         │                    │
      ├──────────────────────────┴────────────────────┤
      │ phím tắt                                      │
  * LCD 16x2 I2C (tùy chọn): 2 dòng không dấu
  * Console: in khi trạng thái đổi (chạy không màn hình / SSH)
"""
from __future__ import annotations

import logging
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .common import Detection, strip_accents
from .geometry import CameraGeometry
from .mission import MissionOutput, St

log = logging.getLogger("display")

FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
    "C:/Windows/Fonts/arial.ttf",
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
    "/Library/Fonts/Arial Bold.ttf",
)

# Màu nền băng trạng thái (RGB)
LEVEL_COLORS = {
    "idle": (70, 76, 86),
    "move": (21, 101, 192),
    "wait": (46, 125, 50),
    "warn": (230, 145, 0),
    "alarm": (198, 40, 40),
    "manual": (106, 27, 154),
}


def lcd_lines(out: MissionOutput, station_names: Dict[int, str]) -> Tuple[str, str]:
    """Nội dung LCD 16x2 (không dấu)."""
    sid = out.station
    st = f"TRAM {sid}" if sid is not None else ""
    s = out.state
    if s in (St.SEARCH, St.FOLLOW, St.APPROACH, St.DEPART):
        l1 = f"DI CHUYEN->{st.replace(' ', '')}"
        l2 = f"Cach {out.distance:.1f} m" if out.distance is not None else "Dang tim tram"
    elif s == St.WAIT:
        l1 = f"DA DEN {st}"
        l2 = f"Bo rac: con {math.ceil(out.countdown or 0)}s"
    elif s == St.OBSTACLE:
        l1, l2 = "GAP VAT CAN!", "Robot dung"
    elif s == St.ESTOP:
        l1, l2 = "DUNG KHAN CAP", "Nhan R de reset"
    elif s == St.FAULT:
        l1, l2 = "LOI HE THONG", strip_accents(out.subtitle)
    elif s == St.LOST:
        l1, l2 = "KHONG THAY TRAM", st
    elif s == St.DONE:
        l1, l2 = "HOAN THANH", "Nhan START"
    elif s == St.MANUAL:
        l1, l2 = "DIEU KHIEN TAY", ""
    else:
        l1, l2 = "SAN SANG", f"Ke tiep: {st}"
    return strip_accents(l1)[:16].ljust(16), strip_accents(l2)[:16].ljust(16)


class Lcd1602:
    def __init__(self, address: int = 0x27, port: int = 1):
        from RPLCD.i2c import CharLCD
        self.lcd = CharLCD("PCF8574", address, port=port, cols=16, rows=2, auto_linebreaks=False)
        self.lcd.clear()
        self._last = ("", "")

    def show(self, l1: str, l2: str) -> None:
        if (l1, l2) == self._last:
            return
        self._last = (l1, l2)
        self.lcd.cursor_pos = (0, 0)
        self.lcd.write_string(l1)
        self.lcd.cursor_pos = (1, 0)
        self.lcd.write_string(l2)

    def close(self) -> None:
        try:
            self.lcd.clear()
            self.lcd.close(clear=True)
        except Exception:
            pass


class Display:
    WIN = "Robot thu gom rac - Xuong may"

    def __init__(self, cfg, gui: bool = True, fullscreen: bool = False, lcd: bool = False):
        self.cfg = cfg
        self.W, self.H = cfg.DISPLAY_SIZE
        self.gui = gui
        self.station_names = {k: v.get("name", f"TRẠM {k}") for k, v in cfg.STATIONS.items()}
        self.geom = CameraGeometry.from_config(cfg)
        self._last_console = ("", 0.0)
        self._fonts: Dict[int, object] = {}
        self._font_path = self._find_font(cfg.FONT_PATH)
        self._unicode_ok = self._font_path is not None
        self.lcd = None
        self.last_canvas = None
        if lcd:
            try:
                self.lcd = Lcd1602(cfg.LCD1602_ADDR)
                log.info("LCD 16x2 I2C sẵn sàng (0x%02X)", cfg.LCD1602_ADDR)
            except Exception as exc:
                log.warning("Không mở được LCD 16x2: %s", exc)
        if self.gui:
            import cv2
            if os.name != "nt" and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
                log.warning("Không có màn hình (DISPLAY) -> chuyển sang chế độ console")
                self.gui = False
            else:
                cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.WIN, self.W, self.H)
                if fullscreen:
                    cv2.setWindowProperty(self.WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # ----------------------------------------------------------------
    @staticmethod
    def _find_font(preferred: str) -> Optional[str]:
        for p in (preferred, *FONT_CANDIDATES):
            if p and os.path.exists(p):
                return p
        log.warning("Không tìm thấy font Unicode -> chữ hiển thị không dấu")
        return None

    def _font(self, size: int):
        from PIL import ImageFont
        f = self._fonts.get(size)
        if f is None:
            f = ImageFont.truetype(self._font_path, size) if self._font_path else ImageFont.load_default()
            self._fonts[size] = f
        return f

    def _txt(self, s: str) -> str:
        return s if self._unicode_ok else strip_accents(s)

    def _fit(self, draw, text: str, max_w: int, size: int, min_size: int = 12):
        text = self._txt(text)
        while size > min_size:
            f = self._font(size)
            if draw.textlength(text, font=f) <= max_w:
                return text, f
            size -= 2
        return text, self._font(min_size)

    # ----------------------------------------------------------------
    def render(self, frame, dets: Sequence[Detection], out: MissionOutput, info: Dict) -> None:
        if self.lcd is not None:
            try:
                self.lcd.show(*lcd_lines(out, self.station_names))
            except Exception as exc:
                log.warning("Lỗi LCD: %s", exc)
                self.lcd = None
        self._console(out, info)
        if not self.gui:
            return
        import cv2
        canvas = self.compose(frame, dets, out, info)
        self.last_canvas = canvas
        cv2.imshow(self.WIN, canvas)

    def compose(self, frame, dets: Sequence[Detection], out: MissionOutput, info: Dict):
        """Dựng ảnh giao diện (BGR numpy) – tách riêng để chụp màn hình / kiểm thử."""
        import numpy as np
        import cv2
        from PIL import Image, ImageDraw

        W, H = self.W, self.H
        banner_h, footer_h = 92, 28
        cam_h = H - banner_h - footer_h
        cam_w = int(cam_h * 4 / 3)
        panel_x = cam_w

        # ---- ảnh camera + khung nhận diện (vẽ bằng OpenCV cho nhanh) ----
        if frame is None:
            cam = np.zeros((cam_h, cam_w, 3), np.uint8)
            cv2.putText(cam, "NO CAMERA", (cam_w // 2 - 90, cam_h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        else:
            cam = cv2.resize(frame, (cam_w, cam_h))
        self._draw_floor(cam, info.get("free"), cam_w, cam_h)
        tgt_cls = self.cfg.STATIONS[out.station]["class"] if out.station in self.cfg.STATIONS else ""
        for d in dets:
            x1, y1, x2, y2 = int(d.x1 * cam_w), int(d.y1 * cam_h), int(d.x2 * cam_w), int(d.y2 * cam_h)
            if d.cls in self.cfg.OBSTACLE_CLASSES:
                color = (0, 0, 255) if (out.obstacle_det is not None and d is out.obstacle_det) else (0, 140, 255)
            elif d.cls == tgt_cls:
                color = (0, 220, 0)
            else:
                color = (200, 200, 200)
            thick = 3 if (d is out.target_det or d is out.obstacle_det) else 1
            cv2.rectangle(cam, (x1, y1), (x2, y2), color, thick)
            label = f"{d.cls} {d.conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ly = y1 if y1 >= th + 6 else y1 + th + 6          # sát mép trên -> ghi nhãn bên trong khung
            cv2.rectangle(cam, (x1, ly - th - 6), (x1 + tw + 4, ly), color, -1)
            cv2.putText(cam, label, (x1 + 2, ly - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.drawMarker(cam, (cam_w // 2, cam_h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)

        # ---- khung tổng (Pillow, chữ tiếng Việt) ----
        img = Image.new("RGB", (W, H), (24, 26, 30))
        img.paste(Image.fromarray(cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)), (0, banner_h))
        draw = ImageDraw.Draw(img)

        color = LEVEL_COLORS.get(out.level, LEVEL_COLORS["idle"])
        if out.level == "alarm" and int(time.monotonic() * 2) % 2:
            color = tuple(int(c * 0.6) for c in color)                 # nhấp nháy khi báo động
        draw.rectangle([0, 0, W, banner_h], fill=color)
        t, f = self._fit(draw, out.title, W - 24, 36)
        draw.text((12, 8), t, font=f, fill=(255, 255, 255))
        t, f = self._fit(draw, out.subtitle, W - 24, 21)
        draw.text((12, 56), t, font=f, fill=(235, 235, 235))

        # ---- bảng bên phải ----
        px, py = panel_x + 12, banner_h + 8
        pw = W - panel_x - 20
        draw.text((px, py), self._txt("LỘ TRÌNH"), font=self._font(16), fill=(160, 170, 180))
        py += 24
        n = max(1, len(out.route))
        r = min(20, max(10, pw // (n * 3)))
        gap = (pw - n * 2 * r) / max(1, n - 1) if n > 1 else 0
        for i, sid in enumerate(out.route):
            cx = int(px + r + i * (2 * r + gap))
            cy = py + r
            current = (i == out.route_idx)
            if current and out.state == St.WAIT:
                fill = LEVEL_COLORS["wait"]
            elif current:
                fill = LEVEL_COLORS["move"]
            else:
                fill = (60, 64, 72)
            draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill,
                         outline=(255, 255, 255) if current else (110, 110, 110), width=2)
            lab = str(sid)
            fw = draw.textlength(lab, font=self._font(r))
            draw.text((cx - fw / 2, cy - r * 0.62), lab, font=self._font(r), fill=(255, 255, 255))
            if i < n - 1:
                draw.line([cx + r + 2, cy, cx + r + gap - 2, cy], fill=(110, 110, 110), width=2)
        py += 2 * r + 12

        if out.state == St.WAIT and out.countdown is not None:
            big = f"{math.ceil(out.countdown)} s"
            f = self._font(64)
            fw = draw.textlength(big, font=f)
            draw.text((panel_x + (W - panel_x - fw) / 2, py), big, font=f, fill=(129, 199, 132))
            py += 76
            draw.text((px, py), self._txt("Mời bỏ rác / vải vụn"), font=self._font(17), fill=(220, 220, 220))
            py += 26
        elif out.distance is not None:
            draw.text((px, py), self._txt(f"Khoảng cách: {out.distance:.2f} m"), font=self._font(18),
                      fill=(144, 202, 249))
            py += 28

        lines: List[Tuple[str, Tuple[int, int, int]]] = []
        mode_txt = {St.MANUAL: "TAY", St.ESTOP: "DỪNG KHẨN"}.get(out.state, "TỰ ĐỘNG")
        lines.append((f"Chế độ: {mode_txt}   Vòng: {out.lap + 1}", (220, 220, 220)))
        vb = info.get("vbat")
        if vb:
            col = (239, 83, 80) if vb < self.cfg.BATTERY_WARN_V else (200, 230, 201)
            lines.append((f"Pin: {vb:.2f} V" + ("  (YẾU!)" if vb < self.cfg.BATTERY_WARN_V else ""), col))
        yaw = info.get("yaw")
        lab = info.get("yaw_label", "Góc quay")
        lines.append((f"{lab}: {yaw:.0f}°" if yaw is not None else f"{lab}: --", (200, 200, 200)))
        rpm = info.get("rpm")
        if rpm:
            lines.append(("RPM  FL {:+.0f}  FR {:+.0f}".format(rpm[0], rpm[1]), (200, 200, 200)))
            lines.append(("     RL {:+.0f}  RR {:+.0f}".format(rpm[2], rpm[3]), (200, 200, 200)))
        lines.append((f"Bộ ĐK động cơ: {info.get('ctrl', '?')}", (200, 200, 200)))
        nav = {"clear": "đường trống", "avoid": "ĐANG LÁCH VẬT CẢN", "blocked": "BỊ CHẶN",
               "none": "chỉ YOLO"}.get(out.nav, "")
        if out.route_kf:
            nav = f"lộ trình {out.route_kf}" + (f" | {nav}" if nav else "")
        if nav:
            lines.append((f"Dẫn đường: {nav}", (144, 202, 249) if out.nav != "avoid" else (255, 183, 77)))
        free = info.get("free")
        if info.get("floor_enabled"):
            if free is not None and getattr(free, "learned", False):
                od = out.obstacle_dist
                lines.append(("Sàn: đã học" + (f" | vật cản {od:.1f} m" if od is not None and od < 5 else ""),
                              (165, 214, 167)))
            else:
                lines.append(("Sàn: CHƯA HỌC – nhấn F khi phía trước trống", (255, 183, 77)))
        if info.get("legs") is not None:
            legs = info["legs"]
            lines.append(("Lộ trình đã dạy: " + (", ".join(f"{a}→{b}" for a, b in legs) if legs else "chưa có"),
                          (200, 200, 200)))
        lines.append((f"Ảnh {info.get('fps', 0):.1f} FPS  YOLO {info.get('yolo_ms', 0):.0f} ms", (200, 200, 200)))
        if info.get("model"):
            lines.append((f"Mô hình: {info['model']}", (160, 170, 180)))
        for w in info.get("warnings", []):
            lines.append((w, (255, 183, 77)))
        for text, col in lines:
            if py > H - footer_h - 20:
                break
            t, f = self._fit(draw, text, pw, 16)
            draw.text((px, py), t, font=f, fill=col)
            py += 22

        # ---- chân trang: phím tắt ----
        draw.rectangle([0, H - footer_h, W, H], fill=(38, 41, 46))
        hint = ("ENTER/G chạy  P dừng  M tay  T dạy lộ trình  F học sàn  N bỏ qua chờ  "
                "K dừng khẩn  R reset  ESC thoát")
        t, f = self._fit(draw, hint, W - 16, 15)
        draw.text((8, H - footer_h + 6), t, font=f, fill=(170, 175, 180))

        return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

    def _draw_floor(self, cam, free, cam_w: int, cam_h: int) -> None:
        """Tô vùng sàn trống, chấm chân vật cản, vẽ làn đường robot + vạch dừng."""
        import numpy as np
        import cv2
        g = self.geom
        y_top = min(1.0, max(0.0, g.row_of_distance(self.cfg.FLOOR_MAX_RANGE_M)))
        if free is not None and getattr(free, "valid", False) and free.mask is not None:
            m = cv2.resize(free.mask, (cam_w, cam_h), interpolation=cv2.INTER_NEAREST).astype(bool)
            m[: int(y_top * cam_h)] = False
            green = np.zeros_like(cam)
            green[:] = (60, 200, 60)
            cam[m] = (0.72 * cam[m] + 0.28 * green[m]).astype(np.uint8)
            for x, y in zip(free.xs, free.rows):
                if np.isfinite(y):
                    cv2.circle(cam, (int(x * cam_w), int(min(y, 0.999) * cam_h)), 3, (0, 0, 255), -1)
        hw = self.cfg.ROBOT_HALF_WIDTH_M + self.cfg.NAV_SAFETY_MARGIN_M
        x_bottom = g.ground_distance(1.0)                       # điểm sàn gần nhất camera thấy
        for side in (-1.0, 1.0):
            pts = []
            for X in (x_bottom, 0.8, 1.1, 1.5, 2.0, 3.0):
                if X < x_bottom:
                    continue
                p = g.image_of_ground(X, side * hw)
                if p is not None:
                    pts.append((int(p[0] * cam_w), int(min(p[1], 0.999) * cam_h)))
            if len(pts) >= 2:
                cv2.polylines(cam, [np.array(pts, np.int32)], False, (0, 200, 255), 1, cv2.LINE_AA)
        a = g.image_of_ground(self.cfg.OBST_STOP_DISTANCE_M, hw)
        b = g.image_of_ground(self.cfg.OBST_STOP_DISTANCE_M, -hw)
        if a and b and a[1] <= 1.0:
            cv2.line(cam, (int(a[0] * cam_w), int(a[1] * cam_h)), (int(b[0] * cam_w), int(b[1] * cam_h)),
                     (0, 0, 220), 1, cv2.LINE_AA)

    # ----------------------------------------------------------------
    def _console(self, out: MissionOutput, info: Dict) -> None:
        msg = f"[{out.state.value:8s}] {out.title} | {out.subtitle}"
        last, t_last = self._last_console
        now = time.monotonic()
        changed = msg.split("|")[0] != last.split("|")[0]
        if changed or now - t_last > 5.0:
            text = msg if self._unicode_ok else strip_accents(msg)
            try:
                print(text, flush=True)
            except UnicodeEncodeError:
                print(strip_accents(msg), flush=True)
            self._last_console = (msg, now)

    def poll_key(self, wait_ms: int = 1) -> int:
        if not self.gui:
            return -1
        import cv2
        k = cv2.waitKey(wait_ms)
        return -1 if k < 0 else (k & 0xFF)

    def close(self) -> None:
        if self.lcd is not None:
            self.lcd.close()
        if self.gui:
            import cv2
            cv2.destroyAllWindows()
