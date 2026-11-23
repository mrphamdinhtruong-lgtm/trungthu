# -*- coding: utf-8 -*-
"""
Sinh file cấu hình C++ cho firmware Arduino từ robot/config.py
và tính "mã băm cấu hình" để Pi kiểm tra firmware trên Arduino có khớp không.

  python flash_arduino.py   -> ghi firmware/robot_firmware/config_gen.h rồi biên dịch + nạp
  main.py                   -> so sánh mã băm Arduino báo về với mã băm tính từ config hiện tại
"""
from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Tuple

PROJECT_DIR = Path(__file__).resolve().parent.parent
FIRMWARE_DIR = PROJECT_DIR / "firmware" / "robot_firmware"
HEADER_PATH = FIRMWARE_DIR / "config_gen.h"
FIRMWARE_SOURCES = ("robot_firmware.ino", "sac_dob.h")


def _f(x: float) -> str:
    """Số thực dạng hằng float C++ (luôn có dấu chấm và hậu tố f)."""
    s = repr(float(x))
    if "e" in s or "E" in s:
        return s + "f"
    if "." not in s:
        s += ".0"
    return s + "f"


def _b(x) -> str:
    return "1" if bool(x) else "0"


def check_pins(cfg) -> None:
    """A4/A5 vừa là I2C (MPU-6050) vừa là dừng khẩn / đo pin -> không được bật cùng lúc."""
    if getattr(cfg, "IMU_ENABLE", False):
        used = [n for n in ("ESTOP_INPUT_ENABLE", "BATTERY_SENSE_ENABLE") if getattr(cfg, n, False)]
        if used:
            raise ValueError("robot/config.py: IMU_ENABLE = True dùng A4 (SDA) / A5 (SCL) cho MPU-6050 nên "
                             + " và ".join(used) + " phải = False")


def header_body(cfg) -> str:
    check_pins(cfg)
    counts_per_rev = cfg.ENCODER_PPR_MOTOR * cfg.GEAR_RATIO * cfg.ENCODER_EDGES
    ratio = (cfg.BATTERY_DIVIDER_R1_K + cfg.BATTERY_DIVIDER_R2_K) / cfg.BATTERY_DIVIDER_R2_K
    lines = [
        "// ---- Truyền thông & thời gian ----",
        f"#define SERIAL_BAUD            {int(cfg.SERIAL_BAUD)}UL",
        f"#define CONTROL_PERIOD_MS      {int(cfg.CONTROL_PERIOD_MS)}",
        f"#define TELEMETRY_PERIOD_MS    {int(cfg.TELEMETRY_PERIOD_MS)}",
        f"#define CMD_TIMEOUT_MS         {int(cfg.CMD_TIMEOUT_MS)}",
        f"#define HW_WATCHDOG            {_b(cfg.HW_WATCHDOG)}",
        "",
        "// ---- Động cơ & encoder ----",
        f"#define RPM_NOMINAL            {_f(cfg.MOTOR_RPM_NOMINAL)}",
        f"#define COUNTS_PER_REV         {_f(counts_per_rev)}",
        f"#define MOTOR_INVERT_FL        {_b(cfg.MOTOR_INVERT[0])}",
        f"#define MOTOR_INVERT_FR        {_b(cfg.MOTOR_INVERT[1])}",
        f"#define MOTOR_INVERT_RL        {_b(cfg.MOTOR_INVERT[2])}",
        f"#define MOTOR_INVERT_RR        {_b(cfg.MOTOR_INVERT[3])}",
        f"#define PWM_DEADZONE           {int(cfg.PWM_DEADZONE)}",
        f"#define PWM_TIMER1_FAST        {_b(cfg.PWM_TIMER1_FAST)}",
        f"#define ACCEL_LIMIT_RPM_S      {_f(cfg.ACCEL_LIMIT_RPM_S)}",
        f"#define SPEED_LPF_ALPHA        {_f(cfg.SPEED_LPF_ALPHA)}",
        f"#define IDLE_SPEED_RPM         {_f(cfg.IDLE_SPEED_RPM)}",
        f"#define DIR_SWITCH_RPM         {_f(cfg.DIR_SWITCH_RPM)}",
        "",
        "// ---- An toàn & pin ----",
        f"#define ESTOP_ENABLE           {_b(cfg.ESTOP_INPUT_ENABLE)}",
        f"#define BATT_ENABLE            {_b(cfg.BATTERY_SENSE_ENABLE)}",
        f"#define BATT_DIVIDER_RATIO     {_f(ratio)}",
        "",
        "// ---- Con quay MPU-6050 (I2C: SDA = A4, SCL = A5) ----",
        f"#define IMU_ENABLE             {_b(getattr(cfg, 'IMU_ENABLE', False))}",
        f"#define IMU_I2C_ADDR           0x{int(getattr(cfg, 'IMU_I2C_ADDR', 0x68)):02X}",
        f"#define IMU_YAW_SIGN           {_f(1.0 if getattr(cfg, 'IMU_YAW_SIGN', 1) >= 0 else -1.0)}",
        f"#define IMU_STILL_DPS          {_f(getattr(cfg, 'IMU_STILL_DPS', 1.0))}",
        "",
        "// ---- Bộ điều khiển SAC + DOB ----",
        f"#define CTRL_MODE_DEFAULT      {int(cfg.CONTROLLER_MODE)}",
        f"#define SAC_TAU_M              {_f(cfg.SAC_TAU_M)}",
        f"#define SAC_GI_E               {_f(cfg.SAC_GAMMA_I[0])}",
        f"#define SAC_GI_X               {_f(cfg.SAC_GAMMA_I[1])}",
        f"#define SAC_GI_U               {_f(cfg.SAC_GAMMA_I[2])}",
        f"#define SAC_GP_E               {_f(cfg.SAC_GAMMA_P[0])}",
        f"#define SAC_GP_X               {_f(cfg.SAC_GAMMA_P[1])}",
        f"#define SAC_GP_U               {_f(cfg.SAC_GAMMA_P[2])}",
        f"#define SAC_SIGMA              {_f(cfg.SAC_SIGMA)}",
        f"#define SAC_K0_E               {_f(cfg.SAC_K0[0])}",
        f"#define SAC_K0_X               {_f(cfg.SAC_K0[1])}",
        f"#define SAC_K0_U               {_f(cfg.SAC_K0[2])}",
        f"#define SAC_KMIN_E             {_f(cfg.SAC_K_MIN[0])}",
        f"#define SAC_KMIN_X             {_f(cfg.SAC_K_MIN[1])}",
        f"#define SAC_KMIN_U             {_f(cfg.SAC_K_MIN[2])}",
        f"#define SAC_KMAX_E             {_f(cfg.SAC_K_MAX[0])}",
        f"#define SAC_KMAX_X             {_f(cfg.SAC_K_MAX[1])}",
        f"#define SAC_KMAX_U             {_f(cfg.SAC_K_MAX[2])}",
        f"#define SAC_E_DEADBAND         {_f(cfg.SAC_E_DEADBAND)}",
        f"#define DOB_KN                 {_f(cfg.DOB_KN)}",
        f"#define DOB_TAU_N              {_f(cfg.DOB_TAU_N)}",
        f"#define DOB_TAU_Q              {_f(cfg.DOB_TAU_Q)}",
        f"#define DOB_LIMIT              {_f(cfg.DOB_LIMIT)}",
        f"#define PI_KP                  {_f(cfg.PI_KP)}",
        f"#define PI_KI                  {_f(cfg.PI_KI)}",
    ]
    return "\n".join(lines) + "\n"


def config_hash(cfg) -> str:
    """Mã băm 8 ký tự của (cấu hình firmware + mã nguồn firmware)."""
    h = hashlib.sha1()
    h.update(header_body(cfg).encode("utf-8"))
    for name in FIRMWARE_SOURCES:
        path = FIRMWARE_DIR / name
        if path.exists():
            # chuẩn hóa xuống dòng để Windows/Linux cho cùng kết quả
            h.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()[:8]


def generate_header(cfg) -> Tuple[str, str]:
    body = header_body(cfg)
    digest = config_hash(cfg)
    text = (
        "// =====================================================================\n"
        "//  FILE TỰ SINH bởi flash_arduino.py từ robot/config.py – KHÔNG SỬA TAY\n"
        "// =====================================================================\n"
        "#ifndef CONFIG_GEN_H\n#define CONFIG_GEN_H\n\n"
        f'#define FW_CONFIG_HASH         "{digest}"\n\n'
        + body
        + "\n#endif  // CONFIG_GEN_H\n"
    )
    return text, digest


def write_header(cfg) -> str:
    text, digest = generate_header(cfg)
    old = HEADER_PATH.read_text(encoding="utf-8") if HEADER_PATH.exists() else None
    if old != text:
        HEADER_PATH.write_text(text, encoding="utf-8", newline="\n")
    return digest
