#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
 NẠP FIRMWARE ARDUINO UNO HOÀN TOÀN BẰNG PYTHON (không cần mở Arduino IDE)
=====================================================================
Các bước tự động:
  1. Đọc robot/config.py -> sinh firmware/robot_firmware/config_gen.h
  2. Tìm arduino-cli (nếu chưa có: --install-cli để tự tải bản chính thức)
  3. Cài core "arduino:avr" nếu thiếu
  4. Biên dịch cho Arduino UNO (arduino:avr:uno)
  5. Tự dò cổng USB và nạp
  6. Mở cổng serial, kiểm tra firmware trả lời đúng mã băm cấu hình

Dùng:
  python flash_arduino.py                    # sinh cấu hình + biên dịch + nạp + kiểm tra
  python flash_arduino.py --install-cli      # lần đầu: tự tải arduino-cli
  python flash_arduino.py --compile-only     # chỉ biên dịch (không cần cắm Arduino)
  python flash_arduino.py --port /dev/ttyUSB0
LƯU Ý: tắt main.py trước khi nạp (cổng serial đang bận sẽ nạp lỗi).
"""
from __future__ import annotations

import argparse
import io
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import time
import urllib.request
import zipfile
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot import fwconfig  # noqa: E402

FQBN = "arduino:avr:uno"
SKETCH_DIR = fwconfig.FIRMWARE_DIR
LOCAL_BIN = PROJECT_DIR / "tools" / "bin"


def info(msg: str) -> None:
    print(f"[flash] {msg}", flush=True)


# ---------------------------------------------------------------------
# arduino-cli
# ---------------------------------------------------------------------
def _cli_asset_name() -> str:
    sysname = platform.system()
    mach = platform.machine().lower()
    if sysname == "Linux":
        if mach in ("aarch64", "arm64"):
            return "Linux_ARM64.tar.gz"
        if mach.startswith("armv7") or mach == "armv6l":
            return "Linux_ARMv7.tar.gz" if mach.startswith("armv7") else "Linux_ARMv6.tar.gz"
        if mach in ("x86_64", "amd64"):
            return "Linux_64bit.tar.gz"
        return "Linux_32bit.tar.gz"
    if sysname == "Windows":
        return "Windows_64bit.zip"
    if sysname == "Darwin":
        return "macOS_ARM64.tar.gz" if mach == "arm64" else "macOS_64bit.tar.gz"
    raise RuntimeError(f"Hệ điều hành chưa hỗ trợ: {sysname} {mach}")


def find_cli(explicit: str = None):
    exe = "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
    candidates = [explicit] if explicit else []
    candidates += [shutil.which("arduino-cli"), str(LOCAL_BIN / exe), str(Path.home() / ".local" / "bin" / exe),
                   str(Path.home() / "bin" / exe)]
    for c in candidates:
        if c and Path(c).exists():
            return c
    return None


def install_cli() -> str:
    """Tải arduino-cli bản mới nhất từ trang chính thức của Arduino vào tools/bin/."""
    asset = _cli_asset_name()
    url = f"https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_{asset}"
    info(f"Tải arduino-cli: {url}")
    with urllib.request.urlopen(url, timeout=120) as r:
        data = r.read()
    LOCAL_BIN.mkdir(parents=True, exist_ok=True)
    exe = "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
    target = LOCAL_BIN / exe
    # chỉ lấy đúng 1 file thực thi trong gói (không giải nén cả thư mục)
    if asset.endswith(".zip"):
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            name = next(n for n in z.namelist() if Path(n).name == exe)
            target.write_bytes(z.read(name))
    else:
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as t:
            member = next(m for m in t.getmembers() if m.isfile() and Path(m.name).name == exe)
            target.write_bytes(t.extractfile(member).read())
    target.chmod(0o755)
    info(f"Đã cài arduino-cli tại {target}")
    return str(target)


def run(cmd, check=True, capture=False):
    info("$ " + " ".join(str(c) for c in cmd))
    res = subprocess.run([str(c) for c in cmd], text=True,
                         stdout=subprocess.PIPE if capture else None,
                         stderr=subprocess.STDOUT if capture else None)
    if check and res.returncode != 0:
        if capture and res.stdout:
            print(res.stdout)
        raise SystemExit(f"[flash] LỖI: lệnh thất bại (mã {res.returncode})")
    return res


def ensure_core(cli: str) -> None:
    res = run([cli, "core", "list"], check=False, capture=True)
    if "arduino:avr" in (res.stdout or ""):
        return
    info("Chưa có core arduino:avr -> cài đặt (cần Internet, chỉ làm 1 lần)")
    run([cli, "core", "update-index"])
    run([cli, "core", "install", "arduino:avr"])


# ---------------------------------------------------------------------
# Kiểm tra sau khi nạp
# ---------------------------------------------------------------------
def verify(port: str, expected: str) -> bool:
    try:
        import serial
    except ImportError:
        info("Chưa cài pyserial -> bỏ qua bước kiểm tra")
        return True
    info(f"Kiểm tra firmware trên {port} ...")
    time.sleep(1.5)
    with serial.Serial(port, cfg.SERIAL_BAUD, timeout=0.2) as ser:
        time.sleep(2.0)                     # UNO reset khi mở cổng
        ser.reset_input_buffer()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            ser.write(b"?\n")
            for _ in range(10):
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if line.startswith("I,ROBOT_FW"):
                    parts = line.split(",")
                    ok = len(parts) > 3 and parts[3] == expected
                    info(f"Arduino trả lời: {line}")
                    info("KHỚP cấu hình – nạp thành công!" if ok else f"SAI mã băm (mong đợi {expected})")
                    return ok
    info("Không nhận được phản hồi từ firmware")
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Sinh cấu hình, biên dịch và nạp firmware Arduino UNO")
    ap.add_argument("--port", default=None, help="cổng Arduino (mặc định: tự dò)")
    ap.add_argument("--cli", default=None, help="đường dẫn arduino-cli")
    ap.add_argument("--install-cli", action="store_true", help="tự tải arduino-cli nếu chưa có")
    ap.add_argument("--compile-only", action="store_true", help="chỉ biên dịch, không nạp")
    ap.add_argument("--no-verify", action="store_true", help="bỏ qua bước kiểm tra sau nạp")
    args = ap.parse_args()

    # 1) Sinh cấu hình
    digest = fwconfig.write_header(cfg)
    info(f"Đã sinh {fwconfig.HEADER_PATH.relative_to(PROJECT_DIR)} (mã băm cấu hình {digest})")

    # 2) arduino-cli
    cli = find_cli(args.cli)
    if cli is None:
        if not args.install_cli:
            print("[flash] Chưa có arduino-cli. Chạy lại với --install-cli để tự tải,\n"
                  "        hoặc cài thủ công: https://arduino.github.io/arduino-cli/latest/installation/")
            return 2
        cli = install_cli()
    info(f"arduino-cli: {cli}")

    # 3) Core AVR
    ensure_core(cli)

    # 4) Biên dịch
    run([cli, "compile", "--fqbn", FQBN, "--warnings", "default", str(SKETCH_DIR)])
    if args.compile_only:
        info("Biên dịch xong (không nạp).")
        return 0

    # 5) Nạp
    port = args.port or (cfg.SERIAL_PORT if cfg.SERIAL_PORT != "auto" else None)
    if port is None:
        from robot.serial_link import find_arduino_port
        port = find_arduino_port()
    from robot.serial_link import describe_ports, port_problem
    if port is None:
        print("[flash] Không tìm thấy Arduino. Cắm cáp USB (cáp có truyền dữ liệu) hoặc dùng --port COMx\n"
              f"        Các cổng đang có:\n{describe_ports()}")
        return 3
    why = port_problem(port)
    if why:                                       # báo dễ hiểu thay vì lỗi khó đọc của avrdude
        print(f"[flash] LỖI CỔNG: {why}\n        Các cổng đang có:\n{describe_ports()}\n"
              "        Xem lại bằng: python -m serial.tools.list_ports -v")
        return 3
    run([cli, "upload", "-p", port, "--fqbn", FQBN, str(SKETCH_DIR)])
    info("Nạp xong.")

    # 6) Kiểm tra
    if not args.no_verify and not verify(port, digest):
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
