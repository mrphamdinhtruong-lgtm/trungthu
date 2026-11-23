#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BÀI 4 – "BỘ NÃO" QUYẾT ĐỊNH CỦA ROBOT: MÁY TRẠNG THÁI  (chạy trên: bất kỳ máy nào, không cần phần cứng)

Học được:
  * Robot luôn ở MỘT trạng thái: TÌM trạm -> TIẾP CẬN -> CHỜ 15 s -> trạm kế tiếp ...
  * Mỗi lần có kết quả nhận diện mới, robot/mission.py gọi update() -> quyết định
    tốc độ (vx tiến, wz quay) + chữ hiện lên màn hình.
  * Ở đây ta TỰ BỊA kết quả nhận diện (như YOLO trả về) để xem robot phản ứng thế nào.

Chạy:  python examples/04_may_trang_thai.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot import config as cfg
from robot.common import Detection
from robot.mission import Mission


def khoi(ten, cx=0.5, cao=0.2):
    """Bịa 1 kết quả YOLO: khối 'ten' ở giữa ảnh theo ngang cx, chiều cao khung 'cao' (0..1)."""
    return Detection(ten, 0.9, cx - cao * 0.4, 0.5 - cao / 2, cx + cao * 0.4, 0.5 + cao / 2)


m = Mission(cfg)
t = 0.0
m.update(t, [])
m.start()                                  # giống nhấn START
truoc = None

kich_ban = (
    [("không thấy gì", [])] * 5 +
    [("thấy tram_1 ở xa, lệch phải", [khoi("tram_1", cx=0.7, cao=0.10)])] * 4 +
    [("tram_1 to dần (robot tiến lại)", [khoi("tram_1", cao=0.10 + 0.04 * k)]) for k in range(12)] +
    [("tram_1 sát trước mặt (~0.37 m)", [khoi("tram_1", cao=0.60)])] * 6 +
    [("đang đứng chờ ở trạm 1", [khoi("tram_1", cao=0.60)])] * 160 +
    [("không thấy gì (đang quay)", [])] * 5 +
    [("thấy tram_2", [khoi("tram_2", cao=0.12)])] * 5 +
    [("tram_1 lọt vào ảnh (không phải đích)", [khoi("tram_1", cao=0.3)])] * 3
)
for mo_ta, ket_qua_yolo in kich_ban:
    t += 0.1                               # mỗi bước = 0.1 giây
    out = m.update(t, ket_qua_yolo)        # <- TOÀN BỘ quyết định nằm ở đây
    if out.countdown is not None and abs(out.countdown - round(out.countdown)) < 0.05 and round(out.countdown) % 5 == 0:
        print(f"t={t:5.1f}s | đếm ngược còn {round(out.countdown):2d} s")
    if (out.state, out.title) != truoc:
        print(f"t={t:5.1f}s | YOLO: {mo_ta:32s} -> {out.state.value:8s} | {out.title}")
        print(f"          lệnh: tiến vx={out.vx:+.2f} m/s, quay wz={out.wz:+.2f} rad/s | {out.subtitle}")
        truoc = (out.state, out.title)
