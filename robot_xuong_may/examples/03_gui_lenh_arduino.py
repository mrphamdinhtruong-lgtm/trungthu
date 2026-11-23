#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BÀI 3 – RASPBERRY PI GỬI LỆNH CHO ARDUINO QUA CÁP USB  (chạy trên: Pi, hoặc laptop có cắm Arduino)

Học được:
  * Pi và Arduino "nói chuyện" bằng các DÒNG CHỮ qua cổng nối tiếp (serial, 115200 baud):
      Pi gửi      W,30,30,30,30   = 4 bánh quay 30 vòng/phút
                  S               = dừng
      Arduino gửi T,ms,rpm1..4,...= tốc độ đo được (20 lần/giây)
  * Pi phải gửi lệnh ĐỀU ĐẶN; ngừng gửi quá 0.5 s thì Arduino tự dừng động cơ (an toàn).

Chạy:
  python examples/03_gui_lenh_arduino.py --fake                 # Arduino GIẢ, không cần phần cứng (Linux/Pi/macOS)
  python examples/03_gui_lenh_arduino.py --port /dev/ttyACM0    # Arduino thật (đã nạp firmware)
!! Với Arduino thật: KÊ ROBOT LÊN cho bánh quay tự do trước khi chạy.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot import config as cfg
from robot import fwconfig
from robot.serial_link import SerialLink

ap = argparse.ArgumentParser()
ap.add_argument("--port", default=cfg.SERIAL_PORT, help='"auto", /dev/ttyACM0, COM5...')
ap.add_argument("--fake", action="store_true", help="dùng Arduino giả lập (cổng serial ảo)")
ap.add_argument("--rpm", type=float, default=30.0)
args = ap.parse_args()

gia = None
if args.fake:
    from robot.fake_arduino import FakeArduino          # chỉ chạy được trên Linux / macOS / Pi
    gia = FakeArduino(cfg, fwconfig.config_hash(cfg)).start()
    args.port = gia.port
    print("Arduino giả ở cổng", gia.port)

link = SerialLink(args.port, cfg.SERIAL_BAUD).open()   # 1) mở cổng, chờ Arduino trả lời
print("Thông tin firmware:", link.info)
try:
    print(f"Gửi W,{args.rpm:.0f},... trong 2 giây (gửi 20 lần/giây):")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        link.send_wheels([args.rpm] * 4)                 # 2) lệnh tốc độ 4 bánh
        time.sleep(0.05)
        tel = link.telemetry                             # 3) dữ liệu Arduino gửi về
    print("  tốc độ đo được (RPM):", [round(x) for x in tel.rpm], "| pin:", tel.vbat, "V")
    print("NGỪNG gửi lệnh 1 giây -> Arduino phải tự dừng:")
    time.sleep(1.0)
    tel = link.telemetry
    print("  tốc độ:", [round(x) for x in tel.rpm], "| cờ mất lệnh:", tel.cmd_timeout)
finally:
    link.stop()                                          # 4) luôn gửi lệnh dừng khi thoát
    link.close()
    if gia:
        gia.stop()
