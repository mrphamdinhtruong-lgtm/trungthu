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
    yaw: Optional[float] = None      # góc quay từ con quay MPU-6050 (độ 0..360, dương = quay trái)
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

    @property
    def yaw_ok(self) -> bool:
        """Arduino báo góc yaw hợp lệ (đã gắn & đọc được MPU-6050)."""
        return bool(self.flags & 0x08) and self.yaw is not None


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


class PortError(RuntimeError):
    """Không mở được cổng serial – thông báo tiếng Việt kèm cách xử lý (in ra cho người dùng)."""


def list_serial_ports() -> list:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return list(list_ports.comports())


def _is_bluetooth(p) -> bool:
    return "bluetooth" in (p.description or "").lower() or "BTHENUM" in (p.hwid or "")


def describe_ports(ports=None) -> str:
    """Danh sách cổng đang có, đánh dấu cổng nào là Arduino / Bluetooth."""
    ports = list_serial_ports() if ports is None else ports
    rows = []
    for p in ports:
        tag = "   <- ARDUINO" if p.vid in KNOWN_VIDS else ("   (Bluetooth – KHÔNG phải Arduino)" if _is_bluetooth(p) else "")
        rows.append(f"    {p.device}: {p.description}{tag}")
    return "\n".join(rows) or "    (không có cổng nào – Arduino chưa cắm hoặc cáp USB không truyền dữ liệu)"


def port_problem(port: str, ports=None) -> Optional[str]:
    """Kiểm tra cổng trước khi dùng. None = không thấy vấn đề; chuỗi = lý do + cách xử lý."""
    up = (port or "").upper()
    if not (up.startswith("COM") or "TTYACM" in up or "TTYUSB" in up or "CU.USB" in up):
        return None                                  # cổng ảo (Arduino giả / pty) -> không kiểm tra
    ports = list_serial_ports() if ports is None else ports
    names = [p.device for p in ports]
    if port not in names:
        return (f"Cổng {port} hiện KHÔNG tồn tại: Arduino chưa cắm, cáp USB lỏng / chỉ sạc, cắm qua cục sạc / hub, "
                "hoặc Arduino đang ở cổng khác (xem danh sách dưới).")
    if names.count(port) > 1:
        return (f"Cổng {port} đang bị 2 thiết bị dùng CHUNG tên (thường là cổng Bluetooth ảo). Đổi số cổng của Arduino: "
                "Device Manager > Ports (COM & LPT) > chuột phải Arduino > Properties > Port Settings > Advanced > "
                "COM Port Number chọn số chưa dùng (vd COM9) > OK, rút cáp cắm lại.")
    return None


def port_help(port: str, exc: Exception, ports=None) -> str:
    """Thông báo lỗi mở cổng dễ hiểu cho người mới."""
    ports = list_serial_ports() if ports is None else ports
    msg = str(exc)
    why = port_problem(port, ports)
    if why is None:
        low = msg.lower()
        if "access" in low or "denied" in low or "busy" in low or "permission" in low:
            why = (f"Cổng {port} đang bị chương trình khác mở (Arduino IDE, Serial Monitor, main.py, motor_test.py...) "
                   "-> tắt chương trình đó rồi chạy lại. Trên Pi/Linux: sudo usermod -aG dialout $USER rồi đăng nhập lại.")
        elif "semaphore" in low or "timeout" in low:
            why = (f"Cổng {port} không phản hồi – thường là cổng Bluetooth ảo trùng tên, hoặc cáp USB chập chờn. "
                   "Rút cáp Arduino ra cắm lại; xem danh sách dưới để chọn đúng cổng có chữ ARDUINO.")
        else:
            why = "Kiểm tra cáp USB, cổng COM và driver."
    return (f"Không mở được cổng {port}: {msg}\n  -> {why}\n  Các cổng đang có:\n{describe_ports(ports)}\n"
            "  Xem lại bằng: python -m serial.tools.list_ports -v")


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
                yaw=float(parts[13]) if len(parts) >= 14 and parts[13] else None,
            )
        if tag == "X" and len(parts) >= 11:
            return "X", DebugSample(
                t_ms=int(parts[1]), motor=int(parts[2]), ref=float(parts[3]), ym=float(parts[4]),
                y=float(parts[5]), u=float(parts[6]), dhat=float(parts[7]),
                K=[float(parts[8]), float(parts[9]), float(parts[10])],
            )
        if tag == "I" and len(parts) >= 6:
            return "I", {"name": parts[1], "version": parts[2], "hash": parts[3],
                         "ts_ms": int(parts[4]), "mode": int(parts[5]),
                         "imu": int(parts[6]) if len(parts) >= 7 and parts[6] else 0}
        if tag == "P" and len(parts) >= 3:
            return "P", (parts[1], float(parts[2]))
        if tag == "A":
            return "A", parts[1:]
    except ValueError:
        return None, None
    return None, None


class SerialLink:
    """Kết nối thật tới Arduino. Có luồng nền đọc telemetry."""

    RECONNECT_S = 2.0                      # mất cáp -> thử mở lại cổng mỗi 2 s

    def __init__(self, port: str = "auto", baud: int = 115200):
        self.port = port
        self._port_spec = port               # "auto" -> lúc nối lại dò lại cổng (số COM có thể đổi)
        self.baud = baud
        self.broken = False                  # True = đang mất kết nối USB, chờ tự nối lại
        self.reconnects = 0
        self._next_retry = 0.0
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
                raise PortError("Không tìm thấy Arduino. Cắm cáp USB (cáp có truyền dữ liệu) hoặc dùng --port.\n"
                                f"  Các cổng đang có:\n{describe_ports()}")
        self.port = port
        log.info("Mở cổng %s @%d", port, self.baud)
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.05, write_timeout=0.5)
        except (serial.SerialException, OSError) as exc:
            raise PortError(port_help(port, exc)) from None
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
    def _io_error(self, what: str, exc: Exception) -> None:
        """Lỗi đọc/ghi = cáp USB rút / chập chờn: báo 1 lần (không in 20 lần/giây), rồi tự nối lại."""
        if not self.broken:
            log.error("Mất kết nối Arduino khi %s (%s) – kiểm tra cáp USB; đang tự kết nối lại mỗi %.0f s...",
                      what, exc, self.RECONNECT_S)
            self.broken = True
            self._next_retry = time.monotonic() + self.RECONNECT_S

    def _try_reconnect(self) -> None:
        import serial  # pyserial
        if time.monotonic() < self._next_retry:
            return
        self._next_retry = time.monotonic() + self.RECONNECT_S
        with self._lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            port = self.port
            if not self._port_spec or self._port_spec == "auto":
                port = find_arduino_port() or self.port
            try:
                self.ser = serial.Serial(port, self.baud, timeout=0.05, write_timeout=0.5)
            except (serial.SerialException, OSError, ValueError):
                self.ser = None
                return
            self.port = port
            self.broken = False
            self.reconnects += 1
        log.warning("Đã kết nối lại Arduino ở %s (lần %d). Robot vẫn DỪNG – khắc phục xong nhấn R để chạy lại",
                    port, self.reconnects)

    def _reader(self) -> None:
        buf = b""
        while not self._stop.is_set():
            if self.broken or self.ser is None:
                if self.broken:
                    self._try_reconnect()
                time.sleep(0.1)
                buf = b""
                continue
            try:
                chunk = self.ser.read(256)
            except Exception as exc:  # cáp bị rút / chập chờn
                self._io_error("đọc", exc)
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
        if self.ser is None or self.broken:          # đang mất kết nối -> bỏ qua, luồng đọc tự nối lại
            return
        err = None
        with self._lock:
            try:
                if self.ser is not None:
                    self.ser.write(text.encode("ascii"))
            except Exception as exc:
                err = exc
        if err is not None:
            self._io_error("ghi", err)

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

    def gyro_zero(self) -> None:
        """Đo lại độ lệch 0 con quay + đặt góc = 0 (robot phải đứng yên ~0.5 s)."""
        self._write("G\n")
