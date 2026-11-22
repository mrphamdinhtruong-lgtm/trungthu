# -*- coding: utf-8 -*-
"""
Giao tiếp Raspberry Pi <-> Arduino UNO qua USB-Serial (ASCII, 115200 8N1).
Xem đặc tả giao thức ở đầu file firmware/robot_firmware/robot_firmware.ino.
"""
from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Sequence

log = logging.getLogger("serial")

# VID USB thường gặp: Arduino chính hãng, CH340 (bản clone), FTDI, CP210x
KNOWN_VIDS = {0x2341: "Arduino", 0x2A03: "Arduino", 0x1A86: "CH340", 0x0403: "FTDI", 0x10C4: "CP210x"}


@dataclass
class Telemetry:
    t_ms: int = 0
    rpm: List[float] = field(default_factory=lambda: [0.0] * 4)
    u: List[float] = field(default_factory=lambda: [0.0] * 4)
    flags: int = 0
    mode: int = 2
    vbat: float = 0.0
    rx_time: float = 0.0

    @property
    def estop(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def cmd_timeout(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def open_loop(self) -> bool:
        return bool(self.flags & 0x04)


@dataclass
class DebugSample:
    """Dòng X,... : dữ liệu chi tiết 1 bánh mỗi chu kỳ điều khiển."""
    t_ms: int
    motor: int
    ref: float
    ym: float
    y: float
    u: float
    dhat: float
    K: List[float]


def find_arduino_port() -> Optional[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid in KNOWN_VIDS:
            return p.device
    for p in ports:
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            return p.device
    return None


def parse_line(line: str):
    """Phân tích 1 dòng từ Arduino -> (loại, dữ liệu). Trả về (None, None) nếu sai định dạng."""
    parts = line.strip().split(",")
    if not parts or not parts[0]:
        return None, None
    tag = parts[0]
    try:
        if tag == "T" and len(parts) >= 13:
            return "T", Telemetry(
                t_ms=int(parts[1]),
                rpm=[float(x) for x in parts[2:6]],
                u=[float(x) for x in parts[6:10]],
                flags=int(parts[10]),
                mode=int(parts[11]),
                vbat=float(parts[12]),
            )
        if tag == "X" and len(parts) >= 11:
            return "X", DebugSample(
                t_ms=int(parts[1]), motor=int(parts[2]), ref=float(parts[3]), ym=float(parts[4]),
                y=float(parts[5]), u=float(parts[6]), dhat=float(parts[7]),
                K=[float(parts[8]), float(parts[9]), float(parts[10])],
            )
        if tag == "I" and len(parts) >= 6:
            return "I", {"name": parts[1], "version": parts[2], "hash": parts[3],
                         "ts_ms": int(parts[4]), "mode": int(parts[5])}
        if tag == "P" and len(parts) >= 3:
            return "P", (parts[1], float(parts[2]))
        if tag == "A":
            return "A", parts[1:]
    except ValueError:
        return None, None
    return None, None


class SerialLink:
    """Kết nối thật tới Arduino. Có luồng nền đọc telemetry."""

    def __init__(self, port: str = "auto", baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.telemetry = Telemetry()
        self.info: Optional[Dict] = None
        self.params: Dict[str, float] = {}
        self.debug: Deque[DebugSample] = deque(maxlen=20000)
        self.acks: Deque[List[str]] = deque(maxlen=50)
        self.on_line: Optional[Callable[[str], None]] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_rx = 0.0

    # ----------------------------------------------------------------
    def open(self, wait_info_s: float = 4.0) -> "SerialLink":
        import serial  # pyserial
        port = self.port
        if not port or port == "auto":
            port = find_arduino_port()
            if port is None:
                raise RuntimeError("Không tìm thấy Arduino. Kiểm tra cáp USB hoặc đặt SERIAL_PORT trong config.py")
        self.port = port
        log.info("Mở cổng %s @%d", port, self.baud)
        self.ser = serial.Serial(port, self.baud, timeout=0.05, write_timeout=0.5)
        # Mở cổng làm UNO tự reset (chân DTR) -> chờ bootloader xong
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="serial-rx", daemon=True)
        self._thread.start()
        t0 = time.monotonic()
        while self.info is None and time.monotonic() - t0 < wait_info_s:
            self._write("?\n")
            time.sleep(0.3)
        if self.info is None:
            log.warning("Arduino không trả lời lệnh '?' – firmware đã nạp chưa? (python flash_arduino.py)")
        else:
            log.info("Firmware: %s", self.info)
        return self

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    # ----------------------------------------------------------------
    def _reader(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as exc:  # cáp bị rút...
                log.error("Lỗi đọc serial: %s", exc)
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="ignore").strip()
                if line:
                    self._handle(line)
            if len(buf) > 1024:
                buf = b""

    def _handle(self, line: str) -> None:
        self._last_rx = time.monotonic()
        if self.on_line:
            try:
                self.on_line(line)
            except Exception:
                pass
        kind, data = parse_line(line)
        if kind == "T":
            data.rx_time = self._last_rx
            self.telemetry = data
        elif kind == "X":
            self.debug.append(data)
        elif kind == "I":
            self.info = data
        elif kind == "P":
            self.params[data[0]] = data[1]
        elif kind == "A":
            self.acks.append(data)
            if data and data[0] == "ERR":
                log.warning("Arduino báo lỗi: %s", ",".join(data))

    def _write(self, text: str) -> None:
        if self.ser is None:
            return
        with self._lock:
            try:
                self.ser.write(text.encode("ascii"))
            except Exception as exc:
                log.error("Lỗi ghi serial: %s", exc)

    # ----------------------------------------------------------------
    # API lệnh
    # ----------------------------------------------------------------
    def alive(self, timeout: float = 1.0) -> bool:
        return self.ser is not None and (time.monotonic() - self._last_rx) < timeout

    def send_wheels(self, rpm: Sequence[float]) -> None:
        self._write("W,%.1f,%.1f,%.1f,%.1f\n" % tuple(rpm))

    def send_open_loop(self, duty: Sequence[float]) -> None:
        self._write("O,%.3f,%.3f,%.3f,%.3f\n" % tuple(duty))

    def stop(self) -> None:
        self._write("S\n")

    def estop(self, active: bool = True) -> None:
        self._write("E,%d\n" % (1 if active else 0))

    def set_controller(self, mode: int) -> None:
        self._write("C,%d\n" % int(mode))

    def reset_gains(self) -> None:
        self._write("R\n")

    def set_param(self, name: str, value: float) -> None:
        self._write("P,%s,%.6g\n" % (name, value))

    def query_params(self, wait_s: float = 1.0) -> Dict[str, float]:
        self.params.clear()
        self._write("Q\n")
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait_s and "mode" not in self.params:
            time.sleep(0.05)
        return dict(self.params)

    def debug_motor(self, idx: int) -> None:
        self._write("D,%d\n" % int(idx))

    def request_info(self) -> None:
        self._write("?\n")
