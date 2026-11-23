# -*- coding: utf-8 -*-
"""La bàn: tự nhận chip theo địa chỉ I2C, đọc đúng thứ tự byte, báo lỗi dễ hiểu (bus I2C giả lập)."""
import math
import struct

import pytest

from robot.compass import Compass, detect_chip


class FakeBus:
    """Bus I2C giả: mỗi địa chỉ là 1 bảng thanh ghi 256 byte. Địa chỉ không có -> OSError như smbus2."""

    def __init__(self, devices):
        self.dev = {a: bytearray(regs) + bytearray(256 - len(regs)) for a, regs in devices.items()}
        self.writes = []

    def _get(self, addr):
        if addr not in self.dev:
            raise OSError(121, "Remote I/O error")
        return self.dev[addr]

    def read_byte(self, addr):
        return self._get(addr)[0]

    def read_byte_data(self, addr, reg):
        return self._get(addr)[reg]

    def read_i2c_block_data(self, addr, reg, n):
        return list(self._get(addr)[reg:reg + n])

    def write_byte_data(self, addr, reg, val):
        self._get(addr)[reg] = val
        self.writes.append((addr, reg, val))

    def close(self):
        pass


def le(*v):
    return struct.pack("<3h", *v)


def be(*v):
    return struct.pack(">3h", *v)


def qmc5883l(x, y, z):
    regs = bytearray(14)
    regs[0:6] = le(x, y, z)
    regs[0x0D] = 0xFF
    return {0x0D: regs}


def qmc5883p(x, y, z):
    regs = bytearray(16)
    regs[0] = 0x80
    regs[1:7] = le(x, y, z)
    return {0x2C: regs}


def hmc5883l(x, y, z):
    regs = bytearray(13)
    regs[3:9] = be(x, z, y)                 # HMC: thứ tự X, Z, Y
    regs[0x0A:0x0D] = b"H43"
    return {0x1E: regs}


@pytest.mark.parametrize("make, chip, addr", [(qmc5883l, "qmc5883l", 0x0D), (qmc5883p, "qmc5883p", 0x2C),
                                              (hmc5883l, "hmc5883l", 0x1E)])
def test_auto_detect_and_read_axes(make, chip, addr, tmp_path):
    bus = FakeBus(make(1200, -340, 77))
    assert detect_chip(bus) == chip
    c = Compass("auto", cal_file=str(tmp_path / "cal.json")).open(bus)
    assert (c.chip, c.addr) == (chip, addr)
    assert c.read_raw() == (1200.0, -340.0, 77.0)
    assert bus.writes and all(a == addr for a, _, _ in bus.writes)      # chỉ cấu hình đúng chip


def test_detect_with_lcd_on_same_bus():
    devices = {0x27: b"\x00"}
    devices.update(qmc5883p(1, 2, 3))
    assert detect_chip(FakeBus(devices)) == "qmc5883p"


def test_no_compass_gives_helpful_message():
    with pytest.raises(RuntimeError, match="0x68 = MPU6050"):
        detect_chip(FakeBus({0x68: b"\x00", 0x27: b"\x00"}))
    with pytest.raises(RuntimeError, match="SDA"):
        detect_chip(FakeBus({}))


def test_unknown_type_rejected():
    with pytest.raises(ValueError):
        Compass("hmc9999")


def test_yaw_increases_when_turning_ccw(tmp_path):
    """Sau hiệu chuẩn (offset, scale, ccw_sign) quay CCW 90° thì hướng phải tăng 90°."""
    c = Compass("qmc5883l", cal_file=str(tmp_path / "cal.json"))
    c.cal.update({"offset": [100.0, -50.0], "scale": [1.0, 2.0], "ccw_sign": 1, "yaw_offset_deg": 0.0})
    pts = [(100 + 400 * math.cos(a), -50 + 200 * math.sin(a)) for a in (0.0, math.pi / 2, math.pi)]
    yaws = [c.compute_yaw(x, y) for x, y in pts]
    assert yaws == pytest.approx([0.0, 90.0, 180.0], abs=1e-6)


def test_background_thread_reports_yaw(tmp_path):
    import time
    bus = FakeBus(qmc5883l(500, 0, 0))
    c = Compass("auto", cal_file=str(tmp_path / "cal.json"), rate_hz=200).start(bus)
    try:
        t0 = time.monotonic()
        while c.yaw_deg is None and time.monotonic() - t0 < 2.0:
            time.sleep(0.01)
        assert c.yaw_deg is not None and c.raw[0] == 500.0
    finally:
        c.stop()
