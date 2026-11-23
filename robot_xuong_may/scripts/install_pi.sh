#!/usr/bin/env bash
# =====================================================================
#  CÀI ĐẶT TRÊN RASPBERRY PI 4 (Raspberry Pi OS Bookworm 64-bit)
#  Chạy:  bash scripts/install_pi.sh
# =====================================================================
set -euo pipefail
cd "$(dirname "$0")/.."

echo "[1/6] Gói hệ thống (camera, font tiếng Việt, I2C)..."
sudo apt update
sudo apt install -y python3-venv python3-pip python3-picamera2 python3-libcamera \
                    fonts-dejavu-core i2c-tools git

echo "[2/6] Bật I2C (LCD 16x2 nếu có; con quay MPU-6050 nối Arduino nên không cần) ..."
sudo raspi-config nonint do_i2c 0 || true

echo "[3/6] Quyền truy cập cổng serial / I2C / GPIO / camera ..."
sudo usermod -aG dialout,i2c,gpio,video "$USER" || true

echo "[4/6] Môi trường Python (.venv, dùng chung picamera2 của hệ thống) ..."
python3 -m venv --system-site-packages .venv
# shellcheck disable=SC1091
source .venv/bin/activate
pip install -U pip wheel
pip install -r requirements.txt

echo "[5/6] arduino-cli + core AVR (biên dịch thử firmware) ..."
python flash_arduino.py --install-cli --compile-only

echo "[6/6] Kiểm thử phần mềm ..."
python -m pytest -q

echo
echo "XONG. Đăng xuất/đăng nhập lại để nhận quyền nhóm mới, rồi:"
echo "  source .venv/bin/activate"
echo "  python flash_arduino.py          # nạp Arduino"
echo "  python main.py --sim             # thử mô phỏng"
