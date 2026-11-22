#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CHẠY ARDUINO GIẢ LẬP (cổng serial ảo) để thử giao tiếp Pi–Arduino bằng dữ liệu giả.

  python tools/fake_arduino.py                   # in cổng ảo, vd: /dev/pts/5 – để chạy, Ctrl+C để tắt
  python tools/fake_arduino.py --drop-after 20   # sau 20 s giả "rút cáp" -> main.py phải báo MẤT KẾT NỐI
  python tools/fake_arduino.py --estop-at 15     # sau 15 s giả nhấn nút dừng khẩn

Ở cửa sổ terminal khác:
  python main.py --port /dev/pts/5 --video thu.mp4
  python tools/motor_test.py --port /dev/pts/5 params
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot import fwconfig  # noqa: E402
from robot.fake_arduino import FakeArduino  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drop-after", type=float, default=None, help="giây – giả mất kết nối")
    ap.add_argument("--estop-at", type=float, default=None, help="giây – giả nhấn dừng khẩn")
    ap.add_argument("--vbat", type=float, default=12.3, help="điện áp pin giả (V)")
    ap.add_argument("--verbose", action="store_true", help="in mọi lệnh nhận được")
    args = ap.parse_args()
    fa = FakeArduino(cfg, fwconfig.config_hash(cfg), drop_after_s=args.drop_after,
                     estop_at_s=args.estop_at, vbat=args.vbat).start()
    print(f"Arduino giả lập đang chạy ở cổng: {fa.port}")
    print(f"  -> python main.py --port {fa.port}        (Ctrl+C để tắt)")
    seen = 0
    try:
        while True:
            time.sleep(0.5)
            if args.verbose:
                for line in fa.rx_lines[seen:]:
                    print("  <-", line)
            seen = len(fa.rx_lines)
            status = "DỪNG KHẨN" if fa.estop else ("MẤT CÁP (giả)" if fa.dropped else "chạy")
            print(f"\r  lệnh nhận: {seen:6d} | RPM {[round(x) for x in fa.rpm]} | {status}      ",
                  end="", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        fa.stop()


if __name__ == "__main__":
    main()
