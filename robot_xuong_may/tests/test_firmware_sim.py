# -*- coding: utf-8 -*-
"""
FIRMWARE ARDUINO CHẠY TRÊN CHIP ATmega328P MÔ PHỎNG (simavr) – kiểm tra cả lúc CHẠY, không chỉ biên dịch.

1. Biên dịch firmware/robot_firmware với Arduino AVR core THẬT (cờ biên dịch giống arduino-cli, board UNO)
2. Nạp file .elf vào ATmega328P ảo, mô hình 4 động cơ đọc PWM + chân chiều quay, tạo xung encoder,
   con quay MPU-6050 giả trên I2C (A4/A5) – chạy 2 lần: CÓ và KHÔNG gắn MPU-6050
3. Gửi lệnh qua UART như Raspberry Pi: ?, W, D, P, Q, E, S, G, lệnh sai
Cần (Linux): apt install gcc-avr avr-libc arduino-core-avr simavr libsimavr-dev libelf-dev
Thiếu công cụ -> bỏ qua (skip).
"""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from robot import config, fwconfig

ROOT = Path(__file__).resolve().parent.parent
CORE = Path("/usr/share/arduino/hardware/arduino/avr")
HIL = Path(__file__).parent / "firmware_sim" / "hil.c"
DEF = ["-mmcu=atmega328p", "-DF_CPU=16000000L", "-DARDUINO=10819", "-DARDUINO_AVR_UNO", "-DARDUINO_ARCH_AVR"]


def _need():
    for tool in ("avr-gcc", "avr-g++", "avr-size", "gcc"):
        if shutil.which(tool) is None:
            pytest.skip(f"thiếu {tool}")
    if not (CORE / "cores" / "arduino" / "Arduino.h").exists():
        pytest.skip("thiếu Arduino AVR core (apt install arduino-core-avr)")
    if not Path("/usr/include/simavr/sim_avr.h").exists():
        pytest.skip("thiếu simavr (apt install simavr libsimavr-dev libelf-dev)")


def build_firmware(out: Path) -> Path:
    inc = ["-I", str(CORE / "cores" / "arduino"), "-I", str(CORE / "variants" / "standard")]
    cfl = ["-c", "-g", "-Os", "-w", "-std=gnu11", "-ffunction-sections", "-fdata-sections", "-flto", "-fno-fat-lto-objects"]
    cxx = ["-c", "-g", "-Os", "-std=gnu++11", "-fpermissive", "-fno-exceptions", "-ffunction-sections",
           "-fdata-sections", "-fno-threadsafe-statics", "-Wno-error=narrowing", "-flto"]
    wire = CORE / "libraries" / "Wire" / "src"
    inc += ["-I", str(wire), "-I", str(wire / "utility")]
    objs = []
    for f in [*sorted((CORE / "cores" / "arduino").iterdir()), wire / "Wire.cpp", wire / "utility" / "twi.c"]:
        o = out / (f.name + ".o")
        if f.suffix == ".c":
            cmd = ["avr-gcc", *cfl, *DEF, *inc, str(f), "-o", str(o)]
        elif f.suffix == ".cpp":
            cmd = ["avr-g++", *cxx, "-w", *DEF, *inc, str(f), "-o", str(o)]
        elif f.suffix == ".S":
            cmd = ["avr-gcc", "-c", "-g", "-x", "assembler-with-cpp", "-flto", *DEF, *inc, str(f), "-o", str(o)]
        else:
            continue
        subprocess.run(cmd, check=True, capture_output=True)
        objs.append(str(o))
    fw = fwconfig.FIRMWARE_DIR
    sk = out / "sketch.o"
    r = subprocess.run(["avr-g++", *cxx, "-Wall", "-Wextra", *DEF, *inc, "-I", str(fw), "-x", "c++",
                        str(fw / "robot_firmware.ino"), "-o", str(sk)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert "warning" not in r.stderr.lower(), r.stderr            # firmware không có cảnh báo
    elf = out / "robot_firmware.elf"
    subprocess.run(["avr-gcc", "-w", "-Os", "-g", "-flto", "-fuse-linker-plugin", "-Wl,--gc-sections",
                    "-mmcu=atmega328p", "-o", str(elf), str(sk), *objs, "-lm"], check=True, capture_output=True)
    return elf


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    _need()
    out = tmp_path_factory.mktemp("fw")
    assert config.IMU_ENABLE, "kiểm thử này cần IMU_ENABLE = True (cấu hình giao cho người dùng)"
    assert fwconfig.HEADER_PATH.read_text(encoding="utf-8") == fwconfig.generate_header(config)[0], \
        "config_gen.h cũ – chạy flash_arduino.py --compile-only hoặc fwconfig.write_header"
    elf = build_firmware(out)
    size = subprocess.run(["avr-size", "-C", "--mcu=atmega328p", str(elf)], capture_output=True, text=True).stdout
    prog = int(re.search(r"Program:\s+(\d+)", size).group(1))
    data = int(re.search(r"Data:\s+(\d+)", size).group(1))
    exe = out / "hil"
    subprocess.run(["gcc", "-O2", "-o", str(exe), str(HIL), "-lsimavr", "-lelf", "-lm"], check=True, capture_output=True)
    return exe, elf, prog, data


class Log(str):
    """Nhật ký chạy + độ lệch thời gian (firmware đo con quay ~0.7 s lúc khởi động)."""
    offset = 0.0


def _run(built, *extra):
    exe, elf, _, _ = built
    r = subprocess.run([str(exe), str(elf), *extra], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stdout[-2000:]
    log = Log(r.stdout)
    log.offset = float(re.search(r"^OFFSET ([\d.]+)", r.stdout, re.M).group(1))
    return log


@pytest.fixture(scope="module", params=["co_mpu6050", "khong_mpu6050"])
def run_log(built, request):
    log = _run(built) if request.param == "co_mpu6050" else _run(built, "--no-imu")
    return log, built[2], built[3]


@pytest.fixture(scope="module")
def imu_log(built):
    return _run(built)


def telemetry(log, t0, t1):
    """Các dòng T trong khoảng [t0, t1] của KỊCH BẢN (đã cộng độ lệch khởi động)."""
    off = getattr(log, "offset", 0.0)
    rows = []
    for m in re.finditer(r"^([\d.]+) <- T,(.*)$", log, re.M):
        t = float(m.group(1))
        if t0 + off <= t <= t1 + off:
            f = m.group(2).split(",")
            rows.append((t, [float(x) for x in f[1:5]], int(f[9]) & 7, [float(x) for x in f[5:9]],
                         int(f[9]), float(f[12])))
    return rows


def test_fits_in_arduino_uno(run_log):
    _, prog, data = run_log
    assert prog < 32256 * 0.75            # bộ nhớ chương trình UNO (trừ bootloader)
    assert data < 2048 * 0.55             # chừa ≥ 900 byte RAM cho ngăn xếp


def test_boot_info_and_config_hash(run_log):
    log = run_log[0]
    first = re.search(r"^([\d.]+) <- (I,ROBOT_FW,[^\n]*)$", log, re.M)
    assert first and float(first.group(1)) < 0.1 + log.offset
    f = first.group(2).split(",")
    assert f[3] == fwconfig.config_hash(config)
    assert int(f[6]) == (0x68 if log.offset else 0)          # WHO_AM_I của MPU-6050 (0 = không gắn)


def test_speed_loop_tracks_50_rpm(run_log):
    rows = telemetry(run_log[0], 1.2, 2.3)
    assert rows and all(45 <= v <= 55 for _, rpm, _, _, _, _ in rows for v in rpm), rows[:3]
    assert all(fl == 0 for _, _, fl, _, _, _ in rows)


def test_command_timeout_stops_motors(run_log):
    rows = telemetry(run_log[0], 2.95, 3.55)
    assert rows and all(fl & 2 for _, _, fl, _, _, _ in rows)   # cờ mất lệnh
    assert all(abs(v) < 6 for v in rows[-1][1])


def test_mecanum_reverse_pattern(run_log):
    rows = telemetry(run_log[0], 3.88, 4.09)                  # W,-40,40,-40,40 trước khi dừng khẩn
    assert rows
    for _, (fl_, fr, rl, rr), _, _, _, _ in rows:
        assert fl_ < -28 and fr > 28 and rl < -28 and rr > 28


def test_estop_and_bad_commands(run_log):
    log = run_log[0]
    assert "<- A,OK,E,1" in log and "<- A,OK,E,0" in log
    rows = telemetry(log, 4.12, 4.69)                          # E,1 lúc 4.10 s, vẫn gửi W tới 4.55 s
    assert rows and all(fl & 1 for _, _, fl, _, _, _ in rows)  # cờ dừng khẩn chốt
    assert all(u == 0.0 for _, _, _, us, _, _ in rows for u in us)   # PWM = 0: lệnh W bị BỎ QUA khi dừng khẩn
    assert all(abs(v) < 2.5 for v in rows[-1][1])               # bánh đã dừng
    assert all(fl == 0 for _, _, fl, _, _, _ in telemetry(log, 4.72, 4.80))  # E,0 xóa được
    assert "<- A,ERR,CMD" in log and "<- A,ERR,W" in log
    assert "<- A,OK,P,gie,7.0000" in log and "<- P,gie,7.0000" in log


# ---------------------------------------------------------------------
#  CON QUAY MPU-6050
# ---------------------------------------------------------------------
def test_imu_yaw_integrates_rotation(imu_log):
    """Kịch bản: quay trái 30 °/s trong 1 s (lúc 1.0–2.0 s) -> góc yaw ≈ +30°, cờ bit3 = góc hợp lệ."""
    before = telemetry(imu_log, 0.40, 0.95)
    after = telemetry(imu_log, 2.10, 2.35)
    assert before and after
    assert all(full & 8 for *_, full, _ in before + after)
    assert all(abs(y) < 0.5 or abs(y - 360) < 0.5 for *_, y in before)     # độ lệch 0 đã được trừ
    assert all(28.5 <= y <= 31.5 for *_, y in after), [r[-1] for r in after]


def test_imu_still_drift_is_compensated(imu_log):
    """Đứng yên (3.0–5.2 s) mà độ lệch 0 của con quay trôi 40 LSB (0.3 °/s) -> góc gần như không đổi."""
    rows = telemetry(imu_log, 3.6, 5.2)
    assert rows
    ys = [y for *_, y in rows]
    assert max(ys) - min(ys) < 1.0, ys


def test_imu_recalibrate_command(imu_log):
    assert "<- A,OK,G" in imu_log
    rows = telemetry(imu_log, 6.0, 6.2)
    assert rows and all(abs(y) < 0.5 or abs(y - 360) < 0.5 for *_, y in rows)


def test_no_imu_flags_and_g_error(run_log):
    """Không gắn MPU-6050: khởi động ngay, cờ bit3 = 0, lệnh G báo lỗi; các kiểm thử tốc độ / mất lệnh /
    dừng khẩn ở trên cũng chạy với kịch bản 'khong_mpu6050' -> điều khiển bánh vẫn bình thường."""
    log = run_log[0]
    if log.offset:
        pytest.skip("kịch bản có MPU-6050")
    rows = telemetry(log, 0.4, 2.3)
    assert rows and all(not (full & 8) for *_, full, _ in rows)
    assert "<- A,ERR,G_IMU" in log
