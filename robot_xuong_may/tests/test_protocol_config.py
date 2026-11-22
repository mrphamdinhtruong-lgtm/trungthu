# -*- coding: utf-8 -*-
import re
from pathlib import Path

from robot import config, fwconfig
from robot.common import strip_accents
from robot.display import lcd_lines
from robot.mission import MissionOutput, St
from robot.serial_link import parse_line

INO = Path(fwconfig.FIRMWARE_DIR / "robot_firmware.ino").read_text(encoding="utf-8")
INO_CODE = re.sub(r'"(?:\\.|[^"\\])*"', '""', INO)          # bỏ chuỗi ký tự
INO_CODE = re.sub(r"//[^\n]*|/\*.*?\*/", "", INO_CODE, flags=re.S)  # bỏ chú thích


def test_parse_telemetry():
    kind, t = parse_line("T,12345,50.1,-49.8,50.0,-50.2,0.52,-0.51,0.50,-0.53,3,2,11.87")
    assert kind == "T"
    assert t.t_ms == 12345 and t.rpm[1] == -49.8 and t.u[3] == -0.53
    assert t.estop and t.cmd_timeout and not t.open_loop
    assert t.mode == 2 and t.vbat == 11.87


def test_parse_info_debug_param_ack_and_garbage():
    assert parse_line("I,ROBOT_FW,1.0.0,abcd1234,20,2")[1]["hash"] == "abcd1234"
    k, x = parse_line("X,100,0,50.0,48.2,47.9,0.612,-0.101,1.002,0.371,0.802")
    assert k == "X" and x.motor == 0 and x.K == [1.002, 0.371, 0.802]
    assert parse_line("P,gie,5.0000") == ("P", ("gie", 5.0))
    assert parse_line("A,ERR,W")[0] == "A"
    assert parse_line("T,1,2,abc") == (None, None)
    assert parse_line("") == (None, None)


def test_header_defines_every_macro_firmware_uses():
    text, digest = fwconfig.generate_header(config)
    defined = set(re.findall(r"#define\s+(\w+)", text))
    # các macro cấu hình firmware dùng nhưng tự định nghĩa trong .ino thì bỏ qua
    local = set(re.findall(r"#define\s+(\w+)", INO_CODE))
    used = set(re.findall(r"\b([A-Z][A-Z0-9_]{3,})\b", INO_CODE))
    candidates = {u for u in used if u not in local and (
        u.startswith(("SAC_", "DOB_", "PI_", "MOTOR_INVERT", "BATT_", "ESTOP_", "PWM_", "CTRL_MODE",
                      "HW_", "FW_CONFIG", "CMD_", "TELEMETRY_", "CONTROL_", "SERIAL_", "RPM_", "COUNTS_",
                      "ACCEL_", "SPEED_", "IDLE_", "DIR_")))}
    missing = candidates - defined
    assert not missing, missing
    assert len(digest) == 8


def test_hash_is_deterministic_and_sensitive():
    import types
    a = fwconfig.config_hash(config)
    assert a == fwconfig.config_hash(config)
    d = {k: getattr(config, k) for k in dir(config) if k.isupper()}
    d["SAC_SIGMA"] = config.SAC_SIGMA + 0.01
    assert fwconfig.config_hash(types.SimpleNamespace(**d)) != a


def test_committed_header_is_up_to_date():
    """config_gen.h trong repo phải khớp config.py (nhắc chạy flash_arduino.py)."""
    text, _ = fwconfig.generate_header(config)
    assert fwconfig.HEADER_PATH.read_text(encoding="utf-8") == text


def test_lcd_lines_ascii_16_chars():
    cases = [
        MissionOutput(state=St.APPROACH, station=2, distance=1.234),
        MissionOutput(state=St.WAIT, station=1, countdown=14.2),
        MissionOutput(state=St.OBSTACLE, station=3),
        MissionOutput(state=St.FAULT, station=1, subtitle="Mất kết nối Arduino (kiểm tra cáp USB)"),
        MissionOutput(state=St.IDLE, station=1),
    ]
    for out in cases:
        l1, l2 = lcd_lines(out, {})
        assert len(l1) == 16 and len(l2) == 16
        assert l1.isascii() and l2.isascii()
    assert lcd_lines(cases[0], {})[0].strip() == "DI CHUYEN->TRAM2"
    assert lcd_lines(cases[1], {})[1].strip() == "Bo rac: con 15s"


def test_strip_accents():
    assert strip_accents("ĐANG DI CHUYỂN ĐẾN TRẠM 1") == "DANG DI CHUYEN DEN TRAM 1"
