# -*- coding: utf-8 -*-
"""
La bàn điện tử QMC5883L (GY-271 đời mới, địa chỉ 0x0D) hoặc HMC5883L (0x1E)
nối I2C của Raspberry Pi:  VCC->3.3V, GND->GND, SDA->GPIO2 (chân 3), SCL->GPIO3 (chân 5).
Bật I2C: sudo raspi-config -> Interface Options -> I2C.

Xuất ra yaw_deg: góc hướng (độ), DƯƠNG khi robot quay NGƯỢC chiều kim đồng hồ.
Hiệu chuẩn (bù từ trường cứng/mềm + chiều quay): python tools/compass_calibrate.py

LƯU Ý: đặt la bàn cao, xa động cơ / pin / L298N ít nhất 10-15 cm; khung thép
trong xưởng làm la bàn lệch – chỉ dùng để GỢI Ý hướng & giữ hướng ngắn hạn.
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

log = logging.getLogger("compass")
PROJECT_DIR = Path(__file__).resolve().parent.parent


def _s16(lo: int, hi: int) -> int:
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


class Compass:
    def __init__(self, chip: str = "qmc5883l", bus: int = 1, cal_file: str = "compass_cal.json",
                 rate_hz: float = 50.0):
        self.chip = chip.lower()
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

    def start(self) -> "Compass":
        from smbus2 import SMBus
        self._bus = SMBus(self.bus_no)
        self._init_chip()
        self.load_calibration()
        self._thread = threading.Thread(target=self._loop, name="compass", daemon=True)
        self._thread.start()
        return self

    def _init_chip(self) -> None:
        b = self._bus
        if self.chip == "qmc5883l":
            self.addr = 0x0D
            b.write_byte_data(self.addr, 0x0B, 0x01)       # SET/RESET period
            b.write_byte_data(self.addr, 0x09, 0x1D)       # liên tục, 200 Hz, ±8 G, OSR 512
        elif self.chip == "hmc5883l":
            self.addr = 0x1E
            b.write_byte_data(self.addr, 0x00, 0x70)       # trung bình 8 mẫu, 15 Hz
            b.write_byte_data(self.addr, 0x01, 0x20)       # độ lợi 1090 LSB/G
            b.write_byte_data(self.addr, 0x02, 0x00)       # đo liên tục
        else:
            raise ValueError(f"COMPASS_TYPE không hỗ trợ: {self.chip}")
        time.sleep(0.05)

    def read_raw(self) -> Tuple[float, float, float]:
        b = self._bus
        if self.chip == "qmc5883l":
            d = b.read_i2c_block_data(self.addr, 0x00, 6)
            return float(_s16(d[0], d[1])), float(_s16(d[2], d[3])), float(_s16(d[4], d[5]))
        d = b.read_i2c_block_data(self.addr, 0x03, 6)     # HMC: X, Z, Y (big-endian)
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
