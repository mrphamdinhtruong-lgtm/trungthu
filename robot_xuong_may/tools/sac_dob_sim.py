#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MÔ PHỎNG SO SÁNH PI / SAC / SAC+DOB trên mô hình động cơ JGB37-520 + L298N
(dùng ĐÚNG thuật toán của firmware – robot/sac_dob.py giống hệt sac_dob.h)

Kịch bản (giống thực tế robot):
  0 s   : đặt 55 RPM (0.5)            2 s : tải tăng đột ngột (nhiễu 0.15)
  4 s   : giảm còn 27.5 RPM            5 s : pin sụt 12.4 V -> 10.8 V, hết tải
  6 s   : đảo chiều -44 RPM

  python tools/sac_dob_sim.py                       # vẽ docs/img/sac_dob_sim.png + bảng IAE
  python tools/sac_dob_sim.py --tau 0.15 --friction 0.15 --out ket_qua.png
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot.sac_dob import CTRL_PI, CTRL_SAC, CTRL_SAC_DOB, MODE_NAMES, MotorModel, SacDobParams, simulate  # noqa: E402


def scenario(Ts: float, T: float = 8.0):
    n = int(T / Ts)
    r, d, vb = [], [], []
    for k in range(n):
        t = k * Ts
        r.append(0.5 if t < 4 else (0.25 if t < 6 else -0.4))
        d.append(0.15 if 2 <= t < 5 else 0.0)
        vb.append(12.4 if t < 5 else 10.8)
    return r, d, vb


def run_all(tau: float, friction: float, kv: float):
    Ts = cfg.CONTROL_PERIOD_MS / 1000.0
    r, d, vb = scenario(Ts)
    res = {}
    for mode in (CTRL_PI, CTRL_SAC, CTRL_SAC_DOB):
        p = SacDobParams.from_config(cfg)
        p.mode = mode
        p.prepare()
        motor = MotorModel(Ts=Ts, tau=tau, friction=friction, kv=kv, deadzone_pwm=cfg.PWM_DEADZONE,
                           rpm_nominal=cfg.MOTOR_RPM_NOMINAL, lpf_alpha=cfg.SPEED_LPF_ALPHA,
                           counts_per_rev=cfg.ENCODER_PPR_MOTOR * cfg.GEAR_RATIO * cfg.ENCODER_EDGES)
        o = simulate(p, r, d, vb, motor, accel_n_per_s=cfg.ACCEL_LIMIT_RPM_S / cfg.MOTOR_RPM_NOMINAL)
        # IAE so với đáp ứng mong muốn (mô hình mẫu tau_m)
        am = math.exp(-Ts / cfg.SAC_TAU_M)
        ym = 0.0
        iae = iae_d = 0.0
        for k in range(len(r)):
            err = abs(ym - o["y_true"][k]) * cfg.MOTOR_RPM_NOMINAL * Ts
            iae += err
            t = k * Ts
            if 2.0 <= t < 3.0 or 5.0 <= t < 6.0:
                iae_d += err
            ym = am * ym + (1 - am) * o["ref"][k]
        o["iae"], o["iae_d"] = iae, iae_d
        res[mode] = o
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tau", type=float, default=0.10, help="hằng số thời gian động cơ thật (s)")
    ap.add_argument("--friction", type=float, default=0.10, help="ma sát Coulomb (chuẩn hóa)")
    ap.add_argument("--kv", type=float, default=1.0, help="hệ số tốc độ/điện áp (chuẩn hóa)")
    ap.add_argument("--out", default=str(PROJECT_DIR / "docs" / "img" / "sac_dob_sim.png"))
    args = ap.parse_args()

    res = run_all(args.tau, args.friction, args.kv)
    print(f"{'Bộ ĐK':10s} {'IAE tổng (RPM·s)':>18s} {'IAE khi có nhiễu':>18s}")
    for mode, o in res.items():
        print(f"{MODE_NAMES[mode]:10s} {o['iae']:18.2f} {o['iae_d']:18.2f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    N = cfg.MOTOR_RPM_NOMINAL
    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    colors = {CTRL_PI: "tab:orange", CTRL_SAC: "tab:green", CTRL_SAC_DOB: "tab:blue"}
    any_o = res[CTRL_SAC_DOB]
    t = any_o["t"]
    ax[0].plot(t, [x * N for x in any_o["ref"]], "k:", lw=1, label="tốc độ đặt (sau giới hạn gia tốc)")
    ax[0].plot(t, [x * N for x in any_o["ym"]], "k--", lw=1, label="mô hình mẫu y_m")
    for mode, o in res.items():
        ax[0].plot(t, [x * N for x in o["y_true"]], color=colors[mode], label=f"{MODE_NAMES[mode]} (IAE {o['iae']:.1f})")
        ax[1].plot(t, o["u"], color=colors[mode], label=MODE_NAMES[mode])
    ax[2].plot(t, res[CTRL_SAC_DOB]["dhat"], color=colors[CTRL_SAC_DOB],
               label="d̂ ước lượng (gồm cả ma sát + sai lệch mô hình)")
    ax[2].step(t, [-0.15 * (2 <= x < 5) for x in t], "k--", lw=1, where="post",
               label="tải thật quy về đầu vào (âm = cản)")
    for key, ls in (("Ke", "-"), ("Kx", "--"), ("Ku", ":")):
        ax[3].plot(t, res[CTRL_SAC_DOB][key], ls, color="tab:blue", label=f"{key} (SAC+DOB)")
        ax[3].plot(t, res[CTRL_SAC][key], ls, color="tab:green", alpha=0.7, label=f"{key} (SAC)")
    for x0, txt in ((2, "tải tăng"), (4, "giảm tốc"), (5, "pin sụt / hết tải"), (6, "đảo chiều")):
        for a in ax:
            a.axvline(x0, color="gray", lw=0.6, alpha=0.5)
        ax[0].text(x0 + 0.03, 62, txt, fontsize=8, color="gray")
    ax[0].set_ylabel("Tốc độ bánh (RPM)")
    ax[1].set_ylabel("u (duty)")
    ax[2].set_ylabel("Nhiễu (chuẩn hóa)")
    ax[3].set_ylabel("Độ lợi thích nghi")
    ax[3].set_xlabel("Thời gian (s)")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="lower left", fontsize=7, ncol=2)
    fig.suptitle(f"PI / SAC / SAC+DOB – động cơ tau={args.tau}s, ma sát={args.friction}, kv={args.kv}")
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"Đã lưu đồ thị: {args.out}")


if __name__ == "__main__":
    main()
