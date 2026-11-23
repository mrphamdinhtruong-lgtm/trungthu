#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
THỬ & HIỆU CHUẨN LA BÀN (QMC5883L / QMC5883P / HMC5883L trên I2C của Raspberry Pi – chạy TRÊN PI)

  python tools/compass_calibrate.py --test     # BƯỚC 1: kiểm tra dây – in loại chip, địa chỉ, số đo, hướng
  python tools/compass_calibrate.py            # BƯỚC 2: robot tự quay chậm 2 vòng (cần Arduino + động cơ)
  python tools/compass_calibrate.py --by-hand  #         hoặc tự xoay robot bằng tay NGƯỢC chiều kim đồng hồ

Kết quả lưu vào compass_cal.json (offset từ trường cứng, hệ số co giãn, chiều quay).
Làm ở đúng nơi robot chạy (trong xưởng), tránh xa vật sắt lớn.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot.compass import Compass  # noqa: E402
from robot.kinematics import MecanumKinematics  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--by-hand", action="store_true", help="xoay robot bằng tay thay vì cho robot tự quay")
    ap.add_argument("--seconds", type=float, default=None, help="thời gian đo (mặc định: hiệu chuẩn 24 s, --test 15 s)")
    ap.add_argument("--wz", type=float, default=0.55, help="tốc độ quay (rad/s) khi robot tự quay")
    ap.add_argument("--port", default=None)
    ap.add_argument("--test", action="store_true", help="chỉ đọc thử la bàn (kiểm tra dây), không hiệu chuẩn")
    args = ap.parse_args()

    comp = Compass(cfg.COMPASS_TYPE, cfg.COMPASS_I2C_BUS, cfg.COMPASS_CAL_FILE)
    try:
        comp.open()
    except ImportError:
        raise SystemExit("Thiếu thư viện smbus2 (pip install smbus2) – lệnh này chạy trên Raspberry Pi")
    except (OSError, RuntimeError) as exc:
        raise SystemExit(f"LỖI LA BÀN: {exc}\n"
                         "  -> bật I2C: sudo raspi-config -> Interface Options -> I2C, khởi động lại\n"
                         "  -> xem địa chỉ: i2cdetect -y 1  (la bàn: 0d / 2c / 1e)")
    print(f"Tìm thấy la bàn {comp.chip.upper()} tại địa chỉ 0x{comp.addr:02X}")
    if args.test:
        return test_read(comp, args.seconds or 15.0)
    args.seconds = args.seconds or 24.0

    link = None
    rpm = None
    if not args.by_hand:
        from robot.serial_link import SerialLink
        link = SerialLink(args.port or cfg.SERIAL_PORT, cfg.SERIAL_BAUD).open()
        rpm = MecanumKinematics.from_config(cfg).inverse(0.0, 0.0, abs(args.wz))
        input("Đặt robot ở chỗ trống, robot sẽ tự QUAY TRÁI ~2 vòng. Enter để bắt đầu...")
    else:
        input("Chuẩn bị xoay robot bằng tay NGƯỢC CHIỀU KIM ĐỒNG HỒ (nhìn từ trên) đều tay ~2 vòng. Enter...")

    xs, ys = [], []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            if link is not None:
                link.send_wheels(rpm)
            x, y, _ = comp.read_raw()
            xs.append(x)
            ys.append(y)
            time.sleep(0.02)
    finally:
        if link is not None:
            link.stop()
            time.sleep(0.1)
            link.close()

    if len(xs) < 50:
        raise SystemExit("Quá ít mẫu – kiểm tra dây I2C (i2cdetect -y 1)")
    ox, oy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    rx, ry = (max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2
    if rx < 1 or ry < 1:
        raise SystemExit("Biên độ từ trường quá nhỏ – robot chưa quay đủ vòng hoặc cảm biến lỗi")
    r = (rx + ry) / 2
    sx, sy = r / rx, r / ry

    # Chiều quay: robot quay CCW thì yaw phải TĂNG
    angs = [math.atan2((y - oy) * sy, (x - ox) * sx) for x, y in zip(xs, ys)]
    total = 0.0
    for a0, a1 in zip(angs, angs[1:]):
        d = a1 - a0
        d = (d + math.pi) % (2 * math.pi) - math.pi
        total += d
    ccw_sign = 1 if total > 0 else -1
    turns = abs(total) / (2 * math.pi)
    cal = {"offset": [ox, oy], "scale": [sx, sy], "ccw_sign": ccw_sign, "yaw_offset_deg": 0.0}
    comp.save_calibration(cal)
    print(f"Đã lưu {comp.cal_path}")
    print(f"  offset = ({ox:.1f}, {oy:.1f})  scale = ({sx:.3f}, {sy:.3f})  ccw_sign = {ccw_sign:+d}")
    print(f"  số vòng đo được ≈ {turns:.2f}  | độ méo elip rx/ry = {rx / ry:.2f}")
    if turns < 1.0:
        print("  !! Chưa đủ 1 vòng – nên hiệu chuẩn lại với --seconds lớn hơn")
    if not 0.7 < rx / ry < 1.4:
        print("  !! Từ trường méo nhiều – dời la bàn xa động cơ / pin / khung sắt")


def test_read(comp: Compass, seconds: float) -> int:
    """In số đo 5 lần/giây. Xoay robot (hoặc module) bằng tay: X, Y phải đổi, hướng phải chạy 0..360."""
    print("Xoay robot từ từ bằng tay. Ctrl+C để dừng.")
    print(f"{'X':>7s} {'Y':>7s} {'Z':>7s} {'|B|':>7s}  hướng (độ)")
    t0, n, bad = time.monotonic(), 0, 0
    xs, ys = [], []
    try:
        while time.monotonic() - t0 < seconds:
            try:
                x, y, z = comp.read_raw()
            except OSError as exc:
                bad += 1
                print("  lỗi đọc I2C:", exc)
                time.sleep(0.2)
                continue
            n += 1
            xs.append(x)
            ys.append(y)
            mag = math.sqrt(x * x + y * y + z * z)
            print(f"{x:7.0f} {y:7.0f} {z:7.0f} {mag:7.0f}  {comp.compute_yaw(x, y):6.1f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    if n == 0:
        print("!! Không đọc được lần nào – kiểm tra dây SDA/SCL")
        return 1
    if max(xs) == min(xs) and max(ys) == min(ys):
        print("!! Số đo đứng yên – chip chưa đo (thử đặt COMPASS_TYPE đúng loại trong robot/config.py)")
        return 1
    print(f"OK: {n} lần đọc, {bad} lỗi. Tiếp theo: python tools/compass_calibrate.py --by-hand")
    if not comp.cal_path.exists():
        print("   (chưa có compass_cal.json nên hướng ở trên CHƯA chính xác – hiệu chuẩn xong mới đúng)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
