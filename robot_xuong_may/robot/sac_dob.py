# -*- coding: utf-8 -*-
"""
Bản Python của bộ điều khiển SAC + DOB – GIỐNG HỆT firmware/robot_firmware/sac_dob.h.

Dùng để:
  * mô phỏng / chỉnh tham số trên máy tính (tools/sac_dob_sim.py),
  * kiểm tra đối chiếu với bản C++ (tests/test_sac_dob_parity.py),
  * làm tài liệu thuật toán cho báo cáo.

Ký hiệu (tín hiệu chuẩn hóa: y = RPM / RPM_danh_dinh, u = duty -1..1):
  Mô hình mẫu   : tau_m * dym/dt = -ym + r
  Sai số        : e = ym - y
  Véc-tơ hồi quy: rho = [e, ym, r]
  SAC           : K_P = Gamma_P * e * rho
                  dK_I/dt = Gamma_I * e * rho - sigma * (K_I - K0)
                  u_sac = (K_P + K_I) . rho
  DOB           : d_hat = Q*Pn^-1*y - Q*u ,  Pn = Kn/(tau_n s+1),  Q = 1/(tau_q s+1)
  Luật điều khiển: u = sat(u_sac - d_hat)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence

CTRL_PI = 0
CTRL_SAC = 1
CTRL_SAC_DOB = 2
MODE_NAMES = {CTRL_PI: "PI", CTRL_SAC: "SAC", CTRL_SAC_DOB: "SAC+DOB"}


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


@dataclass
class SacDobParams:
    Ts: float = 0.02
    tau_m: float = 0.15
    gi: List[float] = field(default_factory=lambda: [5.0, 10.0, 10.0])
    gp: List[float] = field(default_factory=lambda: [1.0, 0.3, 0.3])
    sigma: float = 0.1
    k0: List[float] = field(default_factory=lambda: [1.0, 0.4, 0.8])
    kmin: List[float] = field(default_factory=lambda: [0.0, -1.5, -1.5])
    kmax: List[float] = field(default_factory=lambda: [8.0, 3.0, 3.0])
    e_db: float = 0.01
    kn: float = 0.90
    tau_n: float = 0.10
    tau_q: float = 0.12
    d_lim: float = 0.6
    pi_kp: float = 1.0
    pi_ki: float = 8.0
    mode: int = CTRL_SAC_DOB
    # dẫn xuất
    am: float = 0.0
    aq: float = 0.0
    inv_tau_q: float = 0.0
    inv_kn: float = 0.0

    def prepare(self) -> "SacDobParams":
        """Giống sacdob_prepare() trong C++."""
        if self.tau_m < self.Ts:
            self.tau_m = self.Ts
        if self.tau_q < self.Ts:
            self.tau_q = self.Ts
        if self.tau_n < 0.0:
            self.tau_n = 0.0
        if self.kn < 0.05:
            self.kn = 0.05
        if self.mode not in (CTRL_PI, CTRL_SAC, CTRL_SAC_DOB):
            self.mode = CTRL_SAC_DOB
        self.am = math.exp(-self.Ts / self.tau_m)
        self.aq = self.Ts / (self.tau_q + self.Ts)
        self.inv_tau_q = 1.0 / self.tau_q
        self.inv_kn = 1.0 / self.kn
        return self

    @classmethod
    def from_config(cls, cfg) -> "SacDobParams":
        return cls(
            Ts=cfg.CONTROL_PERIOD_MS / 1000.0,
            tau_m=cfg.SAC_TAU_M,
            gi=list(cfg.SAC_GAMMA_I),
            gp=list(cfg.SAC_GAMMA_P),
            sigma=cfg.SAC_SIGMA,
            k0=list(cfg.SAC_K0),
            kmin=list(cfg.SAC_K_MIN),
            kmax=list(cfg.SAC_K_MAX),
            e_db=cfg.SAC_E_DEADBAND,
            kn=cfg.DOB_KN,
            tau_n=cfg.DOB_TAU_N,
            tau_q=cfg.DOB_TAU_Q,
            d_lim=cfg.DOB_LIMIT,
            pi_kp=cfg.PI_KP,
            pi_ki=cfg.PI_KI,
            mode=cfg.CONTROLLER_MODE,
        ).prepare()

    def as_vector(self) -> List[float]:
        """Thứ tự tham số dùng cho chương trình kiểm tra C++ (tests/host_cpp)."""
        return [self.Ts, self.tau_m, *self.gi, *self.gp, self.sigma, *self.k0,
                *self.kmin, *self.kmax, self.e_db, self.kn, self.tau_n, self.tau_q,
                self.d_lim, self.pi_kp, self.pi_ki, float(self.mode)]


class SacDobCtrl:
    """Một bộ điều khiển tốc độ cho một động cơ."""

    def __init__(self, params: SacDobParams):
        self.p = params
        self.KI = [0.0, 0.0, 0.0]
        self.K = [0.0, 0.0, 0.0]
        self.reset_gains()
        self.hold(0.0)

    def reset_gains(self) -> None:
        self.KI = list(self.p.k0)
        self.K = list(self.p.k0)

    def hold(self, y: float) -> None:
        self.ym = y
        self.yf = y
        self.uf = 0.0
        self.e = 0.0
        self.u = 0.0
        self.dhat = 0.0
        self.pi_int = 0.0
        self.sat = 0

    def update(self, r: float, y: float) -> float:
        p = self.p
        Ts = p.Ts
        uc = 0.0
        self.e = e = self.ym - y

        if p.mode == CTRL_PI:
            err = r - y
            windup = (self.sat > 0 and err > 0.0) or (self.sat < 0 and err < 0.0)
            if not windup:
                self.pi_int += p.pi_ki * Ts * err
            self.pi_int = _clamp(self.pi_int, -1.0, 1.0)
            uc = p.pi_kp * err + self.pi_int
        else:
            rho = (e, self.ym, r)
            adapt = abs(e) > p.e_db
            if (self.sat > 0 and e > 0.0) or (self.sat < 0 and e < 0.0):
                adapt = False
            for i in range(3):
                g = e * rho[i]
                dk = -p.sigma * (self.KI[i] - p.k0[i])
                if adapt:
                    dk += p.gi[i] * g
                self.KI[i] = _clamp(self.KI[i] + Ts * dk, p.kmin[i], p.kmax[i])
                self.K[i] = self.KI[i] + p.gp[i] * g
                uc += self.K[i] * rho[i]

        # DOB
        self.yf += p.aq * (y - self.yf)
        qpinv = (p.tau_n * (y - self.yf) * p.inv_tau_q + self.yf) * p.inv_kn
        self.dhat = _clamp(qpinv - self.uf, -p.d_lim, p.d_lim)
        comp = self.dhat if p.mode == CTRL_SAC_DOB else 0.0

        ur = uc - comp
        self.u = _clamp(ur, -1.0, 1.0)
        self.sat = 1 if ur > 1.0 else (-1 if ur < -1.0 else 0)

        self.uf += p.aq * (self.u - self.uf)
        self.ym = p.am * self.ym + (1.0 - p.am) * r
        return self.u


# ---------------------------------------------------------------------------
# Mô hình động cơ JGB37-520 + L298N để mô phỏng (chuẩn hóa)
# ---------------------------------------------------------------------------
@dataclass
class MotorModel:
    """
    tau * dy/dt = -y + Kv * Veff/12 - Fc*sign(y) - d_load
      Veff = duty_thuc * (Vbat - Vdrop_L298N)
      duty_thuc = (DZ + (255-DZ)*|u|)/255  (bù vùng chết giống firmware)
    Có lượng tử hóa encoder 1 kênh và trễ 1 chu kỳ.
    """
    tau: float = 0.10
    kv: float = 1.0
    vbat: float = 12.0
    vdrop: float = 2.0
    friction: float = 0.10       # ma sát Coulomb (chuẩn hóa)
    deadzone_pwm: int = 28
    rpm_nominal: float = 110.0
    counts_per_rev: float = 1980.0
    Ts: float = 0.02
    substeps: int = 10
    lpf_alpha: float = 0.5

    def __post_init__(self):
        self.y = 0.0
        self.pos_counts = 0.0
        self.last_counts = 0
        self.y_meas = 0.0
        self.u_delay = 0.0

    def duty_from_u(self, u: float) -> float:
        a = abs(u)
        if a < 0.002:
            return 0.0
        dz = self.deadzone_pwm
        a_lin = 0.03
        if a < a_lin:
            pwm = a / a_lin * (dz + (255 - dz) * a_lin)
        else:
            pwm = dz + (255 - dz) * a
        pwm = min(255.0, float(int(pwm + 0.5)))
        return math.copysign(pwm / 255.0, u)

    def step(self, u: float, d_load: float = 0.0) -> float:
        """Áp u (đã tính ở bước trước – trễ 1 mẫu), trả về tốc độ đo chuẩn hóa đã lọc."""
        u_apply = self.u_delay
        self.u_delay = u
        duty = self.duty_from_u(u_apply)
        v_eff = duty * max(0.0, self.vbat - self.vdrop)
        drive = self.kv * v_eff / 12.0
        h = self.Ts / self.substeps
        for _ in range(self.substeps):
            fr = 0.0
            net = drive - d_load
            if abs(self.y) > 1e-4:
                fr = math.copysign(self.friction, self.y)
            elif abs(net) <= self.friction:
                # dính (static friction) – không quay
                self.y = 0.0
                continue
            else:
                fr = math.copysign(self.friction, net)
            dy = (-self.y + net - fr) / self.tau
            self.y += h * dy
            self.pos_counts += self.y * self.rpm_nominal / 60.0 * self.counts_per_rev * h
        counts = int(math.floor(self.pos_counts))
        delta = counts - self.last_counts
        self.last_counts = counts
        rpm = delta * 60.0 / self.counts_per_rev / self.Ts
        y_n = rpm / self.rpm_nominal
        self.y_meas += self.lpf_alpha * (y_n - self.y_meas)
        return self.y_meas


def simulate(params: SacDobParams, r_seq: Sequence[float], d_seq: Sequence[float] = None,
             vbat_seq: Sequence[float] = None, motor: MotorModel = None,
             accel_n_per_s: float = 300.0 / 110.0):
    """Mô phỏng vòng kín 1 động cơ giống firmware (có giới hạn gia tốc tốc độ đặt)."""
    motor = motor or MotorModel(Ts=params.Ts)
    ctrl = SacDobCtrl(params)
    n = len(r_seq)
    d_seq = d_seq if d_seq is not None else [0.0] * n
    out = {"t": [], "r": [], "ref": [], "ym": [], "y": [], "y_true": [], "u": [], "dhat": [],
           "Ke": [], "Kx": [], "Ku": []}
    ref = 0.0
    y = 0.0
    step_max = accel_n_per_s * params.Ts
    for k in range(n):
        if vbat_seq is not None:
            motor.vbat = vbat_seq[k]
        ref += _clamp(r_seq[k] - ref, -step_max, step_max)
        u = ctrl.update(ref, y)
        y = motor.step(u, d_seq[k])
        out["t"].append(k * params.Ts)
        out["r"].append(r_seq[k])
        out["ref"].append(ref)
        out["ym"].append(ctrl.ym)
        out["y"].append(y)
        out["y_true"].append(motor.y)
        out["u"].append(u)
        out["dhat"].append(ctrl.dhat)
        out["Ke"].append(ctrl.K[0])
        out["Kx"].append(ctrl.K[1])
        out["Ku"].append(ctrl.K[2])
    return out
