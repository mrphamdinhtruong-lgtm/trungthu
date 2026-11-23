# -*- coding: utf-8 -*-
"""Thông báo lỗi cổng COM dễ hiểu – các tình huống thật gặp trên Windows (trùng tên Bluetooth, rút cáp)."""
from types import SimpleNamespace as NS

import pytest

from robot.serial_link import PortError, SerialLink, describe_ports, port_help, port_problem

from conftest import ROOT, run_script

BT = "BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}"
ARD = NS(device="COM5", description="Arduino Uno (COM5)", hwid="USB VID:PID=2341:0043", vid=0x2341)
BT5 = NS(device="COM5", description="Standard Serial over Bluetooth link (COM5)", hwid=BT, vid=None)
BT3 = NS(device="COM3", description="Standard Serial over Bluetooth link (COM3)", hwid=BT, vid=None)


def test_duplicate_com_name_with_bluetooth():
    why = port_problem("COM5", [BT3, ARD, BT5])
    assert why and "CHUNG tên" in why and "Device Manager" in why
    text = describe_ports([BT3, ARD, BT5])
    assert "<- ARDUINO" in text and "Bluetooth – KHÔNG phải Arduino" in text


def test_missing_port_after_unplug():
    why = port_problem("COM7", [BT3, BT5])
    assert why and "KHÔNG tồn tại" in why and "cáp USB" in why
    assert port_problem("COM7", [BT3, NS(device="COM7", description="Arduino Uno (COM7)", hwid="", vid=0x2341)]) is None


def test_help_for_busy_port_and_virtual_ports():
    ard7 = NS(device="COM7", description="Arduino Uno (COM7)", hwid="", vid=0x2341)
    msg = port_help("COM7", OSError("could not open port 'COM7': PermissionError(13, 'Access is denied.')"), [ard7])
    assert "chương trình khác" in msg and "list_ports" in msg
    assert port_problem("/dev/pts/3", []) is None          # Arduino giả (pty) không bị kiểm tra


def test_open_missing_port_raises_friendly_error():
    pytest.importorskip("serial")
    with pytest.raises(PortError, match="Không mở được cổng"):
        SerialLink("COM_KHONG_CO_99" if __import__("os").name == "nt" else "/dev/ttyUSB_khong_co").open()


def test_example_prints_vietnamese_message_not_traceback():
    pytest.importorskip("serial")
    r = run_script([ROOT / "examples" / "03_gui_lenh_arduino.py", "--port", "COM77"], timeout=60)
    assert r.returncode != 0
    assert "Traceback" not in r.stderr, r.stderr[-800:]
    assert "Không mở được cổng COM77" in r.stderr
