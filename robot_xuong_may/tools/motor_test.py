#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CÔNG CỤ THỬ ĐỘNG CƠ / CON QUAY / NHẬN DẠNG / CHỈNH SAC-DOB
(chạy trên Pi hoặc laptop nối Arduino, TẮT main.py trước; Windows thêm --port COM5 TRƯỚC tên lệnh)

  python tools/motor_test.py dir                 # quay lần lượt từng bánh -> kiểm tra chiều quay & encoder
  python tools/motor_test.py count --turns 10    # quay tay bánh 10 vòng -> kiểm tra GEAR_RATIO
  python tools/motor_test.py pattern             # tiến/lùi/ngang/quay -> kiểm tra lắp con lăn mecanum
  python tools/motor_test.py ident               # đo vùng chết PWM + nhận dạng Kn, tau_n (kê bánh lên)
  python tools/motor_test.py step --modes 0 1 2  # đáp ứng bậc thang PI / SAC / SAC+DOB -> CSV + đồ thị
  python tools/motor_test.py tune gie 8          # chỉnh tham số khi đang chạy (mất khi reset Arduino)
  python tools/motor_test.py params              # xem tham số hiện tại trên Arduino
  python tools/motor_test.py imu                 # thử con quay MPU-6050: xoay robot TRÁI 90° bằng tay

AN TOÀN: kê robot lên (bánh không chạm sàn) cho dir/ident/step, trừ khi dùng --on-floor.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot.kinematics import MecanumKinematics  # noqa: E402
from robot.sac_dob import MODE_NAMES  # noqa: E402
from robot.serial_link import PortError, SerialLink  # noqa: E402

WHEELS = ("FL", "FR", "RL", "RR")
OUT_DIR = PROJECT_DIR / cfg.LOG_DIR


def connect(port=None) -> SerialLink:
    try:
        link = SerialLink(port or cfg.SERIAL_PORT, cfg.SERIAL_BAUD).open()
    except PortError as exc:
        raise SystemExit(str(exc))
    if link.info is None:
        raise SystemExit("Arduino không trả lời – đã nạp firmware chưa? (python flash_arduino.py)")
    return link


def hold(link, seconds, fn, rate=25.0):
    """Gọi fn(t) đều đặn trong `seconds` giây (giữ tín hiệu còn sống cho Arduino)."""
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t >= seconds:
            return
        fn(t)
        time.sleep(1.0 / rate)


# ---------------------------------------------------------------------
def cmd_dir(link, args):
    duty = args.duty
    print(f"Quay từng bánh TIẾN với duty {duty:.0%} trong 2 s. Bánh phải quay theo chiều đưa robot ĐI TỚI.")
    for i, name in enumerate(WHEELS):
        input(f"\n>> Nhấn Enter để thử bánh {name} ...")
        rpms = []

        def f(t, i=i):
            d = [0.0] * 4
            d[i] = duty
            link.send_open_loop(d)
            if t > 1.0:
                rpms.append(link.telemetry.rpm[i])
        hold(link, 2.0, f)
        link.stop()
        avg = sum(rpms) / max(1, len(rpms))
        others = [abs(x) for k, x in enumerate(link.telemetry.rpm) if k != i]
        print(f"   {name}: tốc độ đo trung bình {avg:+.1f} RPM")
        if abs(avg) < 3:
            print("   !! Không đo được tốc độ: kiểm tra dây encoder (VCC 5V, GND, kênh A) và chân Arduino")
        ans = input("   Bánh có quay theo chiều TIẾN không? [y/n]: ").strip().lower()
        if ans == "n":
            inv = list(cfg.MOTOR_INVERT)
            inv[i] = not inv[i]
            print(f"   -> Sửa robot/config.py: MOTOR_INVERT = {tuple(inv)}  rồi chạy lại flash_arduino.py")
        if max(others, default=0) > 3:
            print("   !! Bánh khác cũng báo tốc độ -> có thể đấu nhầm encoder giữa các bánh")
        time.sleep(0.5)


def cmd_count(link, args):
    print(f"Quay TAY bánh {WHEELS[args.wheel]} đúng {args.turns} vòng (chậm, đều), xong nhấn Enter.")
    print("(không cấp điện động cơ cũng được, chỉ cần Arduino + encoder có điện)")
    revs = [0.0]
    last = [time.monotonic()]
    import threading
    done = threading.Event()

    def integ():
        while not done.is_set():
            now = time.monotonic()
            revs[0] += abs(link.telemetry.rpm[args.wheel]) / 60.0 * (now - last[0])
            last[0] = now
            time.sleep(0.01)
    th = threading.Thread(target=integ, daemon=True)
    th.start()
    input(">> Bắt đầu quay, Enter khi xong...")
    done.set()
    th.join()
    measured = revs[0]
    print(f"Đo được {measured:.2f} vòng (cấu hình GEAR_RATIO = {cfg.GEAR_RATIO})")
    if measured > 0.1:
        ratio = cfg.GEAR_RATIO * measured / args.turns
        print(f"-> GEAR_RATIO thực tế ≈ {ratio:.1f}  (sai lệch {100 * (measured / args.turns - 1):+.1f}%)")


def cmd_pattern(link, args):
    kin = MecanumKinematics.from_config(cfg)
    v, w = args.speed, 0.6
    moves = [("TIẾN", (v, 0, 0)), ("LÙI", (-v, 0, 0)), ("SANG TRÁI", (0, v, 0)),
             ("SANG PHẢI", (0, -v, 0)), ("QUAY TRÁI", (0, 0, w)), ("QUAY PHẢI", (0, 0, -w))]
    input("Đặt robot trên sàn, xung quanh trống 1 m. Enter để bắt đầu...")
    for name, (vx, vy, wz) in moves:
        print(f"  -> {name}")
        rpm = kin.inverse(vx, vy, wz)
        hold(link, 1.5, lambda t: link.send_wheels(rpm))
        hold(link, 0.8, lambda t: link.stop())
    print("Nếu SANG TRÁI/PHẢI bị chạy chéo hoặc xoay -> con lăn mecanum lắp sai (phải thành hình chữ X nhìn từ trên).")


def cmd_ident(link, args):
    print("NHẬN DẠNG ĐỘNG CƠ – kê robot lên để bánh quay tự do" if not args.on_floor
          else "NHẬN DẠNG ĐỘNG CƠ – trên sàn (robot sẽ chạy thẳng ~1 m)")
    input("Enter để bắt đầu...")
    link.query_params()
    dz_old = link.params.get("dz", cfg.PWM_DEADZONE)

    # 1) Vùng chết: tắt bù vùng chết, tăng duty dần
    link.set_param("dz", 0)
    time.sleep(0.2)
    start = [None] * 4
    ramp_s = 6.0

    def ramp(t):
        d = 0.5 * t / ramp_s
        link.send_open_loop([d] * 4)
        for i in range(4):
            if start[i] is None and abs(link.telemetry.rpm[i]) > 3.0:
                start[i] = d
    hold(link, ramp_s, ramp)
    hold(link, 1.0, lambda t: link.stop())
    link.set_param("dz", dz_old)
    print("Duty bắt đầu quay:", ", ".join(f"{WHEELS[i]}={s:.3f}" if s else f"{WHEELS[i]}=?" for i, s in enumerate(start)))
    valid = [s for s in start if s]
    if valid:
        dz = int(round(255 * min(valid) * 0.9))
        print(f"-> Gợi ý PWM_DEADZONE = {dz}  (hiện tại {cfg.PWM_DEADZONE})")

    # 2) Bậc thang vòng hở u = 0.5 cho từng bánh (dùng luồng debug 50 Hz)
    u_step = args.duty
    results = []
    rows = []
    for i in range(4):
        link.debug.clear()
        link.debug_motor(i)
        time.sleep(0.2)

        hold(link, 2.5, lambda t: link.send_open_loop([u_step] * 4))
        hold(link, 1.5, lambda t: link.stop())
        link.debug_motor(-1)
        time.sleep(0.2)
        samples = [s for s in link.debug if s.motor == i]
        if len(samples) < 20:
            print(f"  {WHEELS[i]}: thiếu dữ liệu debug")
            continue
        t0 = samples[0].t_ms
        ts = [(s.t_ms - t0) / 1000.0 for s in samples]
        ys = [s.y / cfg.MOTOR_RPM_NOMINAL for s in samples]
        us = [s.u for s in samples]
        k0 = next((k for k, u in enumerate(us) if abs(u) > 0.01), 0)
        t_start = ts[k0]
        y_ss = sum(y for t, y in zip(ts, ys) if t_start + 1.5 <= t <= t_start + 2.4) / max(
            1, sum(1 for t in ts if t_start + 1.5 <= t <= t_start + 2.4))
        target = 0.632 * y_ss
        tau = None
        for k in range(k0 + 1, len(ts)):
            if ys[k] >= target:
                y0, y1 = ys[k - 1], ys[k]
                frac = (target - y0) / (y1 - y0) if y1 != y0 else 0
                tau = (ts[k - 1] + frac * (ts[k] - ts[k - 1])) - t_start
                break
        kn = y_ss / u_step
        results.append((kn, tau))
        print(f"  {WHEELS[i]}: Kn = {kn:.3f}  tau = {tau if tau else float('nan'):.3f} s  (y_ss = {y_ss * cfg.MOTOR_RPM_NOMINAL:.1f} RPM)")
        rows += [(WHEELS[i], t, u, y * cfg.MOTOR_RPM_NOMINAL) for t, u, y in zip(ts, us, ys)]
    if results:
        kn = sum(r[0] for r in results) / len(results)
        taus = [r[1] for r in results if r[1]]
        tau = sum(taus) / len(taus) if taus else float("nan")
        print(f"\n-> Gợi ý: DOB_KN = {kn:.2f}   DOB_TAU_N = {tau:.3f}   (sửa robot/config.py rồi nạp lại)")
    OUT_DIR.mkdir(exist_ok=True)
    path = OUT_DIR / time.strftime("ident_%Y%m%d_%H%M%S.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wheel", "t_s", "u", "rpm"])
        w.writerows(rows)
    print(f"Dữ liệu: {path}")


def cmd_step(link, args):
    tgt = args.rpm
    profile = [(0.5, 0.0), (2.0, tgt), (1.5, tgt / 2), (2.0, -tgt / 2), (1.0, 0.0)]
    print(f"BẬC THANG VÒNG KÍN – bánh {WHEELS[args.wheel]}, đặt {tgt:.0f} RPM, bộ ĐK: "
          + ", ".join(MODE_NAMES[m] for m in args.modes))
    input("Kê robot lên (hoặc --on-floor có chỗ chạy ~2 m). Enter để bắt đầu...")
    curves = {}
    OUT_DIR.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for mode in args.modes:
        link.set_controller(mode)
        link.reset_gains()
        time.sleep(0.3)
        link.debug.clear()
        link.debug_motor(args.wheel)
        time.sleep(0.1)
        for dur, r in profile:
            hold(link, dur, lambda t, r=r: link.send_wheels([r] * 4))
        link.stop()
        time.sleep(0.3)
        link.debug_motor(-1)
        time.sleep(0.2)
        s = [x for x in link.debug if x.motor == args.wheel]
        if not s:
            print(f"  {MODE_NAMES[mode]}: không có dữ liệu")
            continue
        t0 = s[0].t_ms
        curves[mode] = s
        iae = sum(abs(x.ym - x.y) for x in s) * cfg.CONTROL_PERIOD_MS / 1000.0
        path = OUT_DIR / f"step_{MODE_NAMES[mode].replace('+', '_')}_{stamp}.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "ref_rpm", "ym_rpm", "y_rpm", "u", "dhat", "Ke", "Kx", "Ku"])
            for x in s:
                w.writerow([(x.t_ms - t0) / 1000.0, x.ref, x.ym, x.y, x.u, x.dhat, *x.K])
        print(f"  {MODE_NAMES[mode]:8s}: IAE(ym - y) = {iae:.2f} RPM·s   -> {path.name}")
        time.sleep(1.0)
    link.set_controller(cfg.CONTROLLER_MODE)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        for mode, s in curves.items():
            t0 = s[0].t_ms
            t = [(x.t_ms - t0) / 1000.0 for x in s]
            ax[0].plot(t, [x.y for x in s], label=f"y – {MODE_NAMES[mode]}")
            ax[1].plot(t, [x.u for x in s], label=MODE_NAMES[mode])
            ax[2].plot(t, [x.dhat for x in s], label=MODE_NAMES[mode])
        any_s = next(iter(curves.values()))
        t0 = any_s[0].t_ms
        ax[0].plot([(x.t_ms - t0) / 1000.0 for x in any_s], [x.ym for x in any_s], "k--", label="mô hình mẫu y_m")
        ax[0].set_ylabel("Tốc độ (RPM)")
        ax[1].set_ylabel("u (duty)")
        ax[2].set_ylabel("d̂ (DOB)")
        ax[2].set_xlabel("Thời gian (s)")
        for a in ax:
            a.grid(alpha=0.3)
            a.legend(loc="upper right", fontsize=8)
        fig.suptitle(f"Đáp ứng tốc độ bánh {WHEELS[args.wheel]}")
        fig.tight_layout()
        png = OUT_DIR / f"step_{stamp}.png"
        fig.savefig(png, dpi=120)
        print(f"Đồ thị: {png}")
    except ImportError:
        print("Cài matplotlib để vẽ đồ thị: pip install matplotlib")


def cmd_tune(link, args):
    link.set_param(args.name, args.value)
    time.sleep(0.3)
    print("Phản hồi:", list(link.acks)[-1:] or "không có")
    print("Lưu ý: tham số mất khi reset Arduino – muốn lưu hãy sửa robot/config.py rồi flash_arduino.py")


def cmd_params(link, args):
    p = link.query_params()
    for k, v in p.items():
        print(f"  {k:6s} = {v:g}")


def cmd_imu(link, args):
    """Thử con quay MPU-6050 nối Arduino (A4/A5): kiểm tra dây + chiều dấu IMU_YAW_SIGN."""
    who = (link.info or {}).get("imu", 0)
    if not cfg.IMU_ENABLE:
        raise SystemExit("IMU_ENABLE = False trong robot/config.py -> bật lên rồi nạp lại Arduino")
    if not who:
        raise SystemExit("Arduino KHÔNG thấy MPU-6050. Kiểm tra: VCC->5V, GND->GND, SDA->A4, SCL->A5 "
                         "(không đảo SDA/SCL), rồi nhấn RESET trên Arduino / chạy lại lệnh này.")
    print(f"Thấy MPU-6050: WHO_AM_I = 0x{who:02X}"
          + ("" if who == 0x68 else "  (chip tương thích/nhái – vẫn dùng được nếu góc đo đúng)"))
    input("Đặt robot ĐỨNG YÊN trên sàn, nhấn Enter để đo độ lệch 0 ...")
    link.acks.clear()
    link.gyro_zero()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0 and not any(a[:2] in (["OK", "G"], ["ERR", "G_IMU"], ["ERR", "G_MOVING"])
                                                  for a in link.acks):
        time.sleep(0.05)
    print("Phản hồi:", ",".join(list(link.acks)[-1]) if link.acks else "không có")
    print(f"Trong {args.seconds:.0f} s: xoay robot bằng tay SANG TRÁI (ngược chiều kim đồng hồ, nhìn từ trên) "
          "khoảng 90° rồi giữ yên.")
    yaw = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        tel = link.telemetry
        yaw = tel.yaw if tel.yaw_ok else None
        print(f"\r  góc quay = {yaw:6.1f}°   " if yaw is not None else "\r  góc quay = --- (mất con quay)", end="")
        time.sleep(0.2)
    print()
    if yaw is None:
        raise SystemExit("Không đọc được góc – kiểm tra dây I2C (lỏng / quá dài > 30 cm)")
    d = (yaw + 180.0) % 360.0 - 180.0
    if 45 <= d <= 135:
        print(f"ĐÚNG: quay trái ≈ {d:.0f}° -> góc TĂNG. Giữ IMU_YAW_SIGN = {cfg.IMU_YAW_SIGN}")
    elif -135 <= d <= -45:
        print(f"NGƯỢC DẤU: quay trái mà góc GIẢM ({d:.0f}°). Sửa robot/config.py: IMU_YAW_SIGN = "
              f"{-cfg.IMU_YAW_SIGN} rồi nạp lại Arduino (python flash_arduino.py)")
    else:
        print(f"Góc đo được {d:.0f}° – chưa xoay đủ ~90° hoặc module gắn không nằm ngang. Thử lại.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("dir"); p.add_argument("--duty", type=float, default=0.3)
    p = sub.add_parser("count"); p.add_argument("--wheel", type=int, default=0); p.add_argument("--turns", type=int, default=10)
    p = sub.add_parser("pattern"); p.add_argument("--speed", type=float, default=0.15)
    p = sub.add_parser("ident"); p.add_argument("--duty", type=float, default=0.5); p.add_argument("--on-floor", action="store_true")
    p = sub.add_parser("step")
    p.add_argument("--wheel", type=int, default=0)
    p.add_argument("--rpm", type=float, default=50.0)
    p.add_argument("--modes", type=int, nargs="+", default=[0, 1, 2])
    p.add_argument("--on-floor", action="store_true")
    p = sub.add_parser("tune"); p.add_argument("name"); p.add_argument("value", type=float)
    sub.add_parser("params")
    p = sub.add_parser("imu"); p.add_argument("--seconds", type=float, default=15.0)
    args = ap.parse_args()

    link = connect(args.port)
    try:
        {"dir": cmd_dir, "count": cmd_count, "pattern": cmd_pattern, "ident": cmd_ident,
         "step": cmd_step, "tune": cmd_tune, "params": cmd_params, "imu": cmd_imu}[args.cmd](link, args)
    except KeyboardInterrupt:
        print("\nDừng.")
    finally:
        link.stop()
        time.sleep(0.1)
        link.close()


if __name__ == "__main__":
    main()
