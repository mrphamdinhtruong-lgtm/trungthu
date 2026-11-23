# -*- coding: utf-8 -*-
"""Con quay MPU-6050 nối Arduino (A4/A5): giao thức, đo hành trình, kiểm tra chân, Arduino giả, lệnh thử."""
import math
import os
import time
import types

import pytest

from robot import config, fwconfig
from robot.kinematics import MecanumKinematics
from robot.odometry import Odometry
from robot.serial_link import parse_line

from conftest import ROOT, run_script


def test_parse_telemetry_with_and_without_yaw():
    k, t = parse_line("T,1000,1.0,2.0,3.0,4.0,0.1,0.2,0.3,0.4,8,2,0.00,123.4")
    assert k == "T" and t.yaw == pytest.approx(123.4) and t.yaw_ok
    k, t = parse_line("T,1000,1.0,2.0,3.0,4.0,0.1,0.2,0.3,0.4,0,2,0.00,0.0")
    assert not t.yaw_ok                                   # cờ bit3 = 0: không có MPU-6050
    k, t = parse_line("T,1000,1.0,2.0,3.0,4.0,0.1,0.2,0.3,0.4,8,2,11.90")   # firmware cũ (13 trường)
    assert t.yaw is None and not t.yaw_ok and t.vbat == pytest.approx(11.9)
    k, i = parse_line("I,ROBOT_FW,1.1.0,abcd1234,20,2,104")
    assert i["imu"] == 0x68
    assert parse_line("I,ROBOT_FW,1.0.0,abcd1234,20,2")[1]["imu"] == 0


def test_odometry_prefers_gyro_heading():
    kin = MecanumKinematics.from_config(config)
    odo = Odometry(kin)
    fwd = kin.inverse(0.2, 0.0, 0.0)
    odo.update(fwd, 0.0, 0.0)
    for k in range(1, 11):                                # bánh báo đi thẳng, con quay báo quay trái 90°
        odo.update(fwd, 0.1 * k, 9.0 * k)
    assert math.degrees(odo.th) == pytest.approx(90.0, abs=1e-6)
    assert odo.y > 0.05                                   # quỹ đạo cong sang trái
    th = odo.th
    odo.update(fwd, 1.1, 300.0)                           # nhảy 210° (Arduino reset) -> bỏ qua
    assert abs(odo.th - th) < 0.01
    odo2 = Odometry(kin)                                  # không có con quay -> dùng bánh như cũ
    rot = kin.inverse(0.0, 0.0, 1.0)
    odo2.update(rot, 0.0)
    odo2.update(rot, 0.1)
    assert odo2.th == pytest.approx(0.1, rel=1e-6)


def test_a4_a5_conflict_is_rejected():
    cfg = types.SimpleNamespace(**{k: getattr(config, k) for k in dir(config) if k.isupper()})
    fwconfig.header_body(cfg)                             # cấu hình giao cho người dùng phải hợp lệ
    cfg.ESTOP_INPUT_ENABLE = True
    with pytest.raises(ValueError, match="A4"):
        fwconfig.header_body(cfg)
    cfg.IMU_ENABLE = False
    assert "#define IMU_ENABLE             0" in fwconfig.header_body(cfg)


@pytest.mark.skipif(os.name != "posix", reason="Arduino giả cần pty")
@pytest.mark.parametrize("imu", [True, False])
def test_fake_arduino_reports_yaw(imu):
    pytest.importorskip("serial")
    from robot.fake_arduino import FakeArduino
    from robot.serial_link import SerialLink
    fa = FakeArduino(config, fwconfig.config_hash(config), imu=imu).start()
    link = SerialLink(fa.port, config.SERIAL_BAUD).open(wait_info_s=3.0)
    try:
        assert link.info["imu"] == (0x68 if imu else 0)
        rpm = MecanumKinematics.from_config(config).inverse(0.0, 0.0, 0.8)   # quay trái tại chỗ
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            link.send_wheels(rpm)
            time.sleep(0.05)
        tel = link.telemetry
        if not imu:
            assert not tel.yaw_ok
            return
        assert tel.yaw_ok and 20.0 < tel.yaw < 60.0       # ~0.8 rad/s × ~1 s (có quán tính)
        link.gyro_zero()                                  # đang quay -> từ chối
        time.sleep(0.2)
        assert ["ERR", "G_MOVING"] in list(link.acks)
        link.stop()
        time.sleep(1.0)
        link.gyro_zero()
        time.sleep(0.3)
        assert ["OK", "G"] in list(link.acks) and link.telemetry.yaw < 1.0
    finally:
        link.close()
        fa.stop()


@pytest.mark.skipif(os.name != "posix", reason="Arduino giả cần pty")
def test_motor_test_imu_command():
    pytest.importorskip("serial")
    from robot.fake_arduino import FakeArduino
    fa = FakeArduino(config, fwconfig.config_hash(config)).start()
    try:
        import subprocess
        import sys
        r = subprocess.run([sys.executable, str(ROOT / "tools" / "motor_test.py"), "--port", fa.port, "imu",
                            "--seconds", "1"], input="\n", capture_output=True, encoding="utf-8",
                           env=dict(os.environ, PYTHONIOENCODING="cp1252"), timeout=60)
        assert r.returncode == 0, r.stderr[-800:]
        assert "WHO_AM_I = 0x68" in r.stdout and "A,OK,G" not in r.stderr
        assert "OK,G" in r.stdout                         # đo độ lệch 0 thành công
    finally:
        fa.stop()
