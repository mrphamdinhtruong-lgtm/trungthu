# -*- coding: utf-8 -*-
"""
La bàn điện tử 3 trục trên I2C của RASPBERRY PI (KHÔNG nối vào Arduino – Arduino đã dùng hết chân):
    GY-271 / module la bàn      Raspberry Pi 4 (đánh số chân vật lý)
    VCC  ------------------->   chân 1  (3.3V)
    GND  ------------------->   chân 6  (GND)
    SCL  ------------------->   chân 5  (GPIO3 / SCL)
    SDA  ------------------->   chân 3  (GPIO2 / SDA)
    DRDY ------------------->   để trống
Bật I2C: sudo raspi-config -> Interface Options -> I2C. Kiểm tra: i2cdetect -y 1

Hỗ trợ 3 loại chip hay gặp trên module GY-271 (COMPASS_TYPE = "auto" tự nhận theo địa chỉ):
    0x0D  QMC5883L   (GY-271 đời 2017–2023, phổ biến nhất)
    0x2C  QMC5883P   (GY-271 đời mới 2024+)
    0x1E  HMC5883L   (GY-271 đời cũ / GY-273 – chip Honeywell)

Xuất ra yaw_deg: góc hướng (độ), DƯƠNG khi robot quay NGƯỢC chiều kim đồng hồ.
Thử đọc:   python tools/compass_calibrate.py --test
Hiệu chuẩn (bù từ trường cứng/mềm + chiều quay): python tools/compass_calibrate.py

LƯU Ý: đặt la bàn NẰM NGANG, cao, xa động cơ / pin / L298N ít nhất 10-15 cm; khung thép
trong xưởng làm la bàn lệch – chỉ dùng để GỢI Ý hướng & giữ hướng ngắn hạn.
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

log = logging.getLogger("compass")
PROJECT_DIR = Path(__file__).resolve().parent.parent

SUPPORTED = ("qmc5883l", "qmc5883p", "hmc5883l")
CHIP_ADDR = {"qmc5883l": 0x0D, "qmc5883p": 0x2C, "hmc5883l": 0x1E}
# Thiết bị I2C hay gặp – để báo lỗi dễ hiểu khi không thấy la bàn
KNOWN_I2C = {
    0x0D: "QMC5883L (la bàn GY-271)", 0x2C: "QMC5883P (la bàn GY-271 đời mới)", 0x1E: "HMC5883L (la bàn)",
    0x0C: "AK8963 / AK09916 (la bàn bên trong MPU9250 / ICM20948 – chưa hỗ trợ)",
    0x68: "MPU6050 / MPU9250 / ICM20948 (IMU – chưa hỗ trợ la bàn loại này)",
    0x69: "MPU6050 / MPU9250 / ICM20948 (IMU, AD0 = 1 – chưa hỗ trợ)",
    0x28: "BNO055 (chưa hỗ trợ)", 0x29: "BNO055 (chưa hỗ trợ)", 0x1C: "LIS3MDL (chưa hỗ trợ)",
    0x27: "LCD 16x2 I2C (PCF8574)", 0x3F: "LCD 16x2 I2C (PCF8574A)",
}


def _s16(lo: int, hi: int) -> int:
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


def scan_i2c(bus) -> List[int]:
    """Quét các địa chỉ đang trả lời trên bus I2C (giống i2cdetect)."""
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    return found


def _probe(bus, addr: int, reg: int, n: int = 1) -> Optional[List[int]]:
    try:
        return list(bus.read_i2c_block_data(addr, reg, n))
    except OSError:
        return None


def detect_chip(bus) -> str:
    """Tự nhận loại la bàn theo địa chỉ + thanh ghi định danh. Không thấy -> RuntimeError (kèm gợi ý)."""
    d = _probe(bus, 0x1E, 0x0A, 3)
    if d is not None and bytes(d) == b"H43":                  # HMC5883L: ID 'H','4','3'
        return "hmc5883l"
    d = _probe(bus, 0x2C, 0x00, 1)
    if d is not None and d[0] == 0x80:                        # QMC5883P: chip ID 0x80
        return "qmc5883p"
    if _probe(bus, 0x0D, 0x0D, 1) is not None:                # QMC5883L (ID 0xFF, có bản khác -> chỉ cần trả lời)
        return "qmc5883l"
    if _probe(bus, 0x1E, 0x00, 1) is not None:                # chip "HMC5883L" nhái không có ID
        return "hmc5883l"
    found = scan_i2c(bus)
    if not found:
        raise RuntimeError("Không thấy thiết bị nào trên I2C – kiểm tra dây SDA (chân 3) / SCL (chân 5) / "
                           "VCC 3.3V (chân 1) / GND (chân 6) và đã bật I2C trong raspi-config chưa")
    names = ", ".join(f"0x{a:02X} = {KNOWN_I2C.get(a, 'không rõ')}" for a in found)
    raise RuntimeError(f"Không thấy la bàn được hỗ trợ (0x0D / 0x2C / 0x1E). Trên bus có: {names}")


class Compass:
    def __init__(self, chip: str = "auto", bus: int = 1, cal_file: str = "compass_cal.json",
                 rate_hz: float = 50.0):
        self.chip = chip.lower()
        if self.chip not in SUPPORTED + ("auto",):
            raise ValueError(f"COMPASS_TYPE không hỗ trợ: {chip} (dùng auto / {' / '.join(SUPPORTED)})")
        self.addr: Optional[int] = None
        self.bus_no = bus
        self.cal_path = Path(cal_file)
        if not self.cal_path.is_absolute():
            self.cal_path = PROJECT_DIR / self.cal_path
        self.rate_hz = rate_hz
        self.cal = {"offset": [0.0, 0.0], "scale": [1.0, 1.0], "ccw_sign": -1, "yaw_offset_deg": 0.0}
        self._bus = None
        self._raw: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._fx = None
        self._fy = None
        self._yaw: Optional[float] = None
        self._ok = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ----------------------------------------------------------------
    def load_calibration(self) -> None:
        if self.cal_path.exists():
            try:
                self.cal.update(json.loads(self.cal_path.read_text(encoding="utf-8")))
                log.info("Đã nạp hiệu chuẩn la bàn: %s", self.cal_path.name)
            except Exception as exc:
                log.warning("File hiệu chuẩn la bàn lỗi: %s", exc)
        else:
            log.warning("Chưa hiệu chuẩn la bàn (python tools/compass_calibrate.py)")

    def save_calibration(self, cal: dict) -> None:
        self.cal.update(cal)
        self.cal_path.write_text(json.dumps(self.cal, indent=2), encoding="utf-8")

    def open(self, bus=None) -> "Compass":
        """Mở bus I2C, nhận chip, cấu hình đo liên tục, nạp hiệu chuẩn (chưa chạy luồng đọc)."""
        if bus is None:
            from smbus2 import SMBus
            bus = SMBus(self.bus_no)
        self._bus = bus
        if self.chip == "auto":
            self.chip = detect_chip(bus)
        self._init_chip()
        log.info("La bàn %s tại địa chỉ 0x%02X (I2C bus %d)", self.chip.upper(), self.addr, self.bus_no)
        self.load_calibration()
        return self

    def start(self, bus=None) -> "Compass":
        self.open(bus)
        self._thread = threading.Thread(target=self._loop, name="compass", daemon=True)
        self._thread.start()
        return self

    def _init_chip(self) -> None:
        b = self._bus
        self.addr = CHIP_ADDR[self.chip]
        if self.chip == "qmc5883l":
            b.write_byte_data(self.addr, 0x0B, 0x01)       # SET/RESET period
            b.write_byte_data(self.addr, 0x09, 0x1D)       # liên tục, 200 Hz, ±8 G, OSR 512
        elif self.chip == "qmc5883p":
            b.write_byte_data(self.addr, 0x0B, 0x08)       # SET/RESET bật, dải ±8 G
            try:
                b.write_byte_data(self.addr, 0x29, 0x06)   # dấu các trục X/Y/Z (theo ví dụ datasheet)
            except OSError:
                pass
            b.write_byte_data(self.addr, 0x0A, 0xCD)       # chế độ normal, 200 Hz, lọc OSR 8 / DSR 8
        else:                                              # hmc5883l
            b.write_byte_data(self.addr, 0x00, 0x70)       # trung bình 8 mẫu, 15 Hz
            b.write_byte_data(self.addr, 0x01, 0x20)       # độ lợi 1090 LSB/G
            b.write_byte_data(self.addr, 0x02, 0x00)       # đo liên tục
        time.sleep(0.05)

    def read_raw(self) -> Tuple[float, float, float]:
        b = self._bus
        if self.chip == "qmc5883l":                        # 0x00..0x05: X, Y, Z (byte thấp trước)
            d = b.read_i2c_block_data(self.addr, 0x00, 6)
            return float(_s16(d[0], d[1])), float(_s16(d[2], d[3])), float(_s16(d[4], d[5]))
        if self.chip == "qmc5883p":                        # 0x01..0x06: X, Y, Z (byte thấp trước)
            d = b.read_i2c_block_data(self.addr, 0x01, 6)
            return float(_s16(d[0], d[1])), float(_s16(d[2], d[3])), float(_s16(d[4], d[5]))
        d = b.read_i2c_block_data(self.addr, 0x03, 6)     # HMC: X, Z, Y (byte cao trước)
        return float(_s16(d[1], d[0])), float(_s16(d[5], d[4])), float(_s16(d[3], d[2]))

    def _loop(self) -> None:
        period = 1.0 / self.rate_hz
        errors = 0
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                x, y, z = self.read_raw()
                self._raw = (x, y, z)
                a = 0.3                                     # lọc thông thấp
                self._fx = x if self._fx is None else self._fx + a * (x - self._fx)
                self._fy = y if self._fy is None else self._fy + a * (y - self._fy)
                self._yaw = self.compute_yaw(self._fx, self._fy)
                self._ok = True
                errors = 0
            except Exception as exc:
                errors += 1
                if errors in (1, 50):
                    log.warning("Lỗi đọc la bàn: %s", exc)
                if errors > 25:
                    self._ok = False
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def compute_yaw(self, x: float, y: float) -> float:
        c = self.cal
        xc = (x - c["offset"][0]) * c["scale"][0]
        yc = (y - c["offset"][1]) * c["scale"][1]
        ang = math.degrees(math.atan2(yc, xc))
        return (c["ccw_sign"] * ang + c.get("yaw_offset_deg", 0.0)) % 360.0

    @property
    def yaw_deg(self) -> Optional[float]:
        return self._yaw if self._ok else None

    @property
    def raw(self) -> Tuple[float, float, float]:
        return self._raw

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
