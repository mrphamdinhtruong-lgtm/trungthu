# -*- coding: utf-8 -*-
"""Kiểm thử thuật toán SAC-DOB (bản Python) và đối chiếu với bản C++ của firmware."""
import random
import shutil
import subprocess
from pathlib import Path

import pytest

from robot.sac_dob import (CTRL_PI, CTRL_SAC, CTRL_SAC_DOB, MotorModel, SacDobCtrl, SacDobParams,
                           simulate)

from conftest import make_cfg

HOST_SRC = Path(__file__).parent / "host_cpp" / "sac_dob_host.cpp"


def params(mode=CTRL_SAC_DOB, **kw):
    p = SacDobParams.from_config(make_cfg())
    p.mode = mode
    for k, v in kw.items():
        setattr(p, k, v)
    return p.prepare()


def step_profile(Ts=0.02, T=6.0, load=0.12):
    n = int(T / Ts)
    r = [0.5 if k * Ts < 4 else 0.25 for k in range(n)]
    d = [load if 2.0 <= k * Ts < 3.5 else 0.0 for k in range(n)]
    return r, d


@pytest.mark.parametrize("mode", [CTRL_PI, CTRL_SAC, CTRL_SAC_DOB])
def test_tracks_step_without_steady_error(mode):
    r, d = step_profile()
    o = simulate(params(mode), r, d)
    # xác lập trước khi có tải và khi ĐANG có tải (2 s .. 3.5 s): sai số < 4 %
    assert abs(o["y_true"][int(1.9 / 0.02)] - 0.5) < 0.02
    assert abs(o["y_true"][int(3.45 / 0.02)] - 0.5) < 0.02
    assert abs(o["y_true"][-1] - 0.25) < 0.02
    assert max(abs(u) for u in o["u"]) <= 1.0


def test_dob_rejects_load_better_than_pi_and_sac():
    r, d = step_profile()
    def iae(mode):
        o = simulate(params(mode), r, d)
        return sum(abs(0.5 - y) for t, y in zip(o["t"], o["y_true"]) if 2.0 <= t < 3.0)
    assert iae(CTRL_SAC_DOB) < iae(CTRL_SAC)
    assert iae(CTRL_SAC_DOB) < iae(CTRL_PI)


def test_gains_bounded_and_no_drift_over_long_run():
    rnd = random.Random(0)
    p = params()
    r = []
    while len(r) < 3000:                       # 60 s ngẫu nhiên
        r += [rnd.choice([0.0, 0.2, 0.4, -0.3, 0.6])] * rnd.randint(40, 150)
    o = simulate(p, r[:3000], [0.0] * 3000)
    for key, lo, hi in (("Ke", p.kmin[0], p.kmax[0] + 1), ("Kx", p.kmin[1] - 1, p.kmax[1] + 1),
                        ("Ku", p.kmin[2] - 1, p.kmax[2] + 1)):
        assert all(lo <= x <= hi for x in o[key])
    tv = sum(abs(a - b) for a, b in zip(o["u"], o["u"][1:])) / 60.0
    assert tv < 1.5                            # không rung / dao động


def test_hold_resets_state_but_keeps_learned_gains():
    c = SacDobCtrl(params())
    for _ in range(200):
        c.update(0.5, 0.3)
    learned = list(c.KI)
    assert learned != list(c.p.k0)
    c.hold(0.1)
    assert c.ym == 0.1 and c.u == 0 and c.dhat == 0
    assert c.KI == learned
    c.reset_gains()
    assert c.KI == list(c.p.k0)


def _build_host(tmp_path):
    gxx = shutil.which("g++") or shutil.which("c++") or shutil.which("clang++")
    if gxx is None:
        pytest.skip("không có trình biên dịch C++")
    exe = tmp_path / "sac_dob_host"
    subprocess.run([gxx, "-O2", "-std=c++11", "-o", str(exe), str(HOST_SRC)], check=True)
    return exe


@pytest.mark.parametrize("mode", [CTRL_PI, CTRL_SAC, CTRL_SAC_DOB])
def test_cpp_firmware_matches_python(tmp_path, mode):
    """sac_dob.h (firmware Arduino) phải cho kết quả giống robot/sac_dob.py."""
    exe = _build_host(tmp_path)
    p = params(mode)
    rnd = random.Random(42)
    motor = MotorModel(Ts=p.Ts)
    seq, y = [], 0.0
    r = 0.0
    for k in range(600):
        if k % 120 == 0:
            r = rnd.choice([0.0, 0.3, 0.6, -0.4, 0.9])
        y = motor.step(r * 0.9, 0.05 if 200 < k < 320 else 0.0)   # y "đo" bất kỳ
        seq.append((r, y))
    lines = [" ".join(repr(float(v)) for v in p.as_vector())]
    ops = []
    for k, (r, y) in enumerate(seq):
        if k == 300:
            lines.append(f"H {y!r}")
            ops.append(("H", y))
        lines.append(f"U {r!r} {y!r}")
        ops.append(("U", r, y))
    res = subprocess.run([str(exe)], input="\n".join(lines) + "\n", text=True, capture_output=True, check=True)
    cpp = [list(map(float, ln.split())) for ln in res.stdout.strip().splitlines()]

    c = SacDobCtrl(params(mode))
    py = []
    for op in ops:
        if op[0] == "H":
            c.hold(op[1])
        else:
            u = c.update(op[1], op[2])
            py.append([u, c.ym, c.e, c.dhat, *c.K])
    assert len(cpp) == len(py) == 600
    worst = max(abs(a - b) for rc, rp in zip(cpp, py) for a, b in zip(rc, rp))
    assert worst < 2e-3, worst                  # float32 (C++) so với float64 (Python)
