# -*- coding: utf-8 -*-
"""
ARDUINO GIẢ LẬP qua cổng serial ảo (pty) – nói ĐÚNG giao thức của firmware/robot_firmware.ino.

Dùng để thử phần giao tiếp Pi–Arduino khi chưa có / chưa nạp Arduino:
  python tools/fake_arduino.py                  # in ra cổng ảo, vd /dev/pts/5
  python main.py --port /dev/pts/5 --video thu.mp4
Mô phỏng: trả lời '?', W/O/S/E/C/R/P/Q/D, gửi telemetry T mỗi 50 ms, bánh xe bám tốc độ đặt
(quán tính bậc 1), TỰ DỪNG khi mất lệnh > CMD_TIMEOUT_MS, chốt dừng khẩn, mất cáp giả (--drop-after).
Chỉ chạy trên Linux / macOS (cần pty).
"""
from __future__ import annotations

import os
import select
import threading
import time
from typing import List, Optional


class FakeArduino:
    def __init__(self, cfg, config_hash: str, tau: float = 0.12, drop_after_s: Optional[float] = None,
                 estop_at_s: Optional[float] = None, vbat: float = 12.3):
        import tty
        self.cfg = cfg
        self.hash = config_hash
        self.tau = tau
        self.drop_after_s = drop_after_s
        self.estop_at_s = estop_at_s
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)                       # không echo, không xử lý dòng
        self.port = os.ttyname(self.slave)
        self.target: List[float] = [0.0] * 4
        self.rpm: List[float] = [0.0] * 4
        self.duty: List[float] = [0.0] * 4
        self.open_loop = False
        self.estop = False
        self.mode = int(cfg.CONTROLLER_MODE)
        self.vbat = vbat
        self.params = {"gie": cfg.SAC_GAMMA_I[0], "taum": cfg.SAC_TAU_M, "kn": cfg.DOB_KN}
        self.debug_motor = -1
        self.rx_lines: List[str] = []
        self._buf = b""
        self._t0 = time.monotonic()
        self._last_cmd = -1e9
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.dropped = False

    # ----------------------------------------------------------------
    def start(self) -> "FakeArduino":
        self._thread = threading.Thread(target=self._run, name="fake-arduino", daemon=True)
        self._thread.start()
        self._send(self._info())
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def _ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000)

    def _info(self) -> str:
        return f"I,ROBOT_FW,fake,{self.hash},{int(self.cfg.CONTROL_PERIOD_MS)},{self.mode}"

    def _send(self, line: str) -> None:
        if self.dropped:
            return
        try:
            os.write(self.master, (line + "\n").encode("ascii"))
        except OSError:
            pass

    # ----------------------------------------------------------------
    def _handle(self, line: str) -> None:
        self.rx_lines.append(line)
        parts = line.split(",")
        c = parts[0][:1] if parts[0] else ""
        try:
            vals = [float(x) for x in parts[1:]] if c not in ("P",) else []
        except ValueError:
            self._send("A,ERR," + (c or "CMD"))
            return
        now = time.monotonic()
        if c == "W" and len(vals) == 4:
            self._last_cmd = now
            if not self.estop:
                lim = self.cfg.MOTOR_RPM_NOMINAL
                self.target = [max(-lim, min(lim, v)) for v in vals]
                self.open_loop = False
        elif c == "O" and len(vals) == 4:
            self._last_cmd = now
            if not self.estop:
                self.duty = [max(-1.0, min(1.0, v)) for v in vals]
                self.open_loop = True
        elif c == "S":
            self._last_cmd = now
            self.target = [0.0] * 4
            self.duty = [0.0] * 4
            self.open_loop = False
        elif c == "E" and vals:
            self.estop = vals[0] > 0.5
            self._send(f"A,OK,E,{1 if self.estop else 0}")
        elif c == "C" and vals:
            self.mode = int(vals[0] + 0.5)
            self._send(f"A,OK,C,{self.mode}")
        elif c == "R":
            self._send("A,OK,R")
        elif c == "P" and len(parts) >= 3:
            try:
                self.params[parts[1]] = float(parts[2])
                self._send(f"A,OK,P,{parts[1]},{float(parts[2]):.4f}")
            except ValueError:
                self._send("A,ERR,P")
        elif c == "Q":
            for k, v in self.params.items():
                self._send(f"P,{k},{v:.4f}")
            self._send(f"P,mode,{float(self.mode):.4f}")
            self._send("A,OK,Q")
        elif c == "D" and vals:
            self.debug_motor = int(vals[0]) if 0 <= vals[0] <= 3 else -1
            self._send(f"A,OK,D,{self.debug_motor}")
        elif c == "?":
            self._send(self._info())
        else:
            self._send("A,ERR,CMD")

    def _run(self) -> None:
        dt = self.cfg.CONTROL_PERIOD_MS / 1000.0
        tel_every = max(1, int(round(self.cfg.TELEMETRY_PERIOD_MS / self.cfg.CONTROL_PERIOD_MS)))
        k = 0
        nxt = time.monotonic()
        while not self._stop.is_set():
            r, _, _ = select.select([self.master], [], [], max(0.0, nxt - time.monotonic()))
            if r:
                try:
                    self._buf += os.read(self.master, 256)
                except OSError:
                    break
                while b"\n" in self._buf:
                    raw, self._buf = self._buf.split(b"\n", 1)
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line:
                        self._handle(line)
            if time.monotonic() < nxt:
                continue
            nxt += dt
            k += 1
            el = time.monotonic() - self._t0
            if self.drop_after_s is not None and el > self.drop_after_s:
                self.dropped = True                      # "rút cáp": không gửi gì nữa
            if self.estop_at_s is not None and el > self.estop_at_s:
                self.estop = True                        # giả nhấn nút 1 lần (nhấn R / E,0 để xóa)
                self.estop_at_s = None
            timeout = (time.monotonic() - self._last_cmd) * 1000.0 > self.cfg.CMD_TIMEOUT_MS
            if timeout or self.estop:
                self.target = [0.0] * 4
                self.duty = [0.0] * 4
                self.open_loop = False
            goal = ([d * self.cfg.MOTOR_RPM_NOMINAL for d in self.duty] if self.open_loop else self.target)
            a = min(1.0, dt / self.tau)
            for i in range(4):
                self.rpm[i] += a * (goal[i] - self.rpm[i])
                if self.estop:
                    self.rpm[i] *= 0.5
            if k % tel_every == 0:
                flags = (1 if self.estop else 0) | (2 if timeout else 0) | (4 if self.open_loop else 0)
                u = [x / self.cfg.MOTOR_RPM_NOMINAL for x in self.rpm]
                self._send("T,%d,%s,%s,%d,%d,%.2f" % (
                    self._ms(), ",".join("%.1f" % x for x in self.rpm), ",".join("%.2f" % x for x in u),
                    flags, self.mode, self.vbat))
