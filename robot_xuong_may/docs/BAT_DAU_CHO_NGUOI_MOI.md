# BẮT ĐẦU CHO NGƯỜI MỚI – FILE NÀO CHẠY Ở ARDUINO, FILE NÀO CHẠY Ở MÁY TÍNH NHÚNG

Tài liệu này dành cho người **mới làm xử lý ảnh và lập trình máy tính nhúng**. Đọc theo thứ tự từ trên xuống.
Mỗi phần có so sánh với thiết bị tự động hóa quen thuộc (PLC, HMI, camera công nghiệp) cho dễ hình dung.

---

## 1. Robot có 3 "bộ não" – mỗi cái làm một việc

```
 ┌──────────────────────┐   chép thư mục    ┌──────────────────────────────┐  cáp USB   ┌────────────────────────┐
 │ LAPTOP của bạn        │  robot_xuong_may  │ RASPBERRY PI 4 (máy tính nhúng)│ (serial)  │ ARDUINO UNO (vi điều   │
 │ Windows / Linux       │ ────────────────► │ Linux + Python                │ ◄───────► │ khiển) – C++           │
 │                       │                   │                               │  W,30,..  │                        │
 │ • viết / sửa code     │                   │ • camera → xử lý ảnh (YOLO)   │  T,rpm..  │ • đọc 4 encoder         │
 │ • chạy mô phỏng       │                   │ • quyết định đi đâu, dừng     │           │ • 4 vòng SAC-DOB 50 lần/s│
 │ • xuất mô hình ONNX   │                   │ • màn hình hiển thị           │           │ • băm xung PWM → L298N  │
 │ • kiểm tra dữ liệu    │                   │ • gửi lệnh tốc độ xuống Arduino│           │ • tự dừng khi mất lệnh  │
 └──────────────────────┘                   └──────────────────────────────┘           └────────────────────────┘
```

| Bộ não | Giống thiết bị công nghiệp nào | Ngôn ngữ | Hệ điều hành | Chương trình nằm ở đâu |
|---|---|---|---|---|
| **Laptop** | máy tính kỹ sư (chỗ cài TIA Portal / GX Works) | Python | Windows/Linux | thư mục `robot_xuong_may/` |
| **Raspberry Pi 4** | máy tính công nghiệp (IPC) + camera thông minh + HMI | Python | Linux (Raspberry Pi OS) | thư mục `~/robot_xuong_may/` trên thẻ nhớ |
| **Arduino UNO** | PLC nhỏ / bộ điều khiển tốc độ động cơ | C++ | không có | chip ATmega328P (giữ cả khi mất điện) |

* **Raspberry Pi** là một **máy tính thật** (có Linux, có thể cắm màn hình, bàn phím). Code Python của bạn chạy trên đó
  như chạy trên laptop. Pi đủ mạnh để xử lý ảnh nhưng **không chạy đúng thời gian thực** (đôi lúc bị chậm vài chục ms).
* **Arduino** là **vi điều khiển**: chỉ chạy **một** chương trình C++ lặp mãi (`loop()`), rất đều đặn → hợp để điều khiển
  động cơ 50 lần/giây. Arduino không chạy được Python (chỉ có 2 KB RAM).
* Hai bên nói chuyện bằng **các dòng chữ** qua cáp USB (giống giao tiếp HMI ↔ PLC bằng giao thức ASCII):
  Pi gửi `W,30,30,30,30` (4 bánh 30 vòng/phút), Arduino gửi lại `T,...` (tốc độ đo được, điện áp pin).

---

## 2. Bảng tra: file nào chạy ở đâu

Ký hiệu cột "Sửa?": ✏️ thường sửa · 🔧 sửa khi cần · 🚫 không cần sửa / không được sửa tay

### A. File chạy trên ARDUINO UNO (C++) – thư mục `firmware/robot_firmware/`

| File | Làm gì | Sửa? |
|---|---|---|
| `robot_firmware.ino` | Chương trình chính của Arduino: đọc encoder bằng ngắt, chạy 4 bộ điều khiển SAC-DOB mỗi 20 ms, nhận lệnh `W/S/E…` từ Pi, gửi telemetry `T,…`, tự dừng khi mất lệnh 0.5 s, watchdog | 🔧 hiếm |
| `sac_dob.h` | Thuật toán điều khiển tốc độ SAC + DOB (giống khối PID trong PLC nhưng thích nghi) | 🚫 (trừ khi nghiên cứu thuật toán) |
| `config_gen.h` | Thông số cho Arduino – **TỰ SINH** từ `robot/config.py` khi chạy `flash_arduino.py` | 🚫 không sửa tay |

**Đưa xuống Arduino thế nào?** Không cần Arduino IDE. Cắm Arduino vào Pi (hoặc laptop) rồi chạy
`python flash_arduino.py` – Python tự sinh `config_gen.h`, biên dịch và nạp (giống "Download to PLC").
Nạp một lần là xong; Arduino giữ chương trình kể cả khi tắt nguồn. Chỉ nạp lại khi sửa nhóm **[B]**, **[C]** trong `config.py`
hoặc sửa file `.ino`.

### B. File chạy trên RASPBERRY PI 4 (Python) – chép **cả thư mục** `robot_xuong_may/` sang Pi

| File | Làm gì (lời thường) | Sửa? |
|---|---|---|
| `main.py` | **Chương trình chính** – lệnh `python main.py` là robot làm việc. Mỗi vòng (~10 lần/s): lấy ảnh → nhận diện → quyết định → gửi lệnh → vẽ màn hình | 🔧 |
| `robot/config.py` | **Bảng tham số** của cả hệ thống (giống DB tham số trong PLC): chân Arduino, kích thước xe, tên lớp, thời gian chờ 15 s, ngưỡng… | ✏️ |
| `robot/camera.py` | Đọc camera (Camera Module 2 / webcam / video / thư mục ảnh), tự mở lại khi mất hình | 🚫 |
| `robot/detector.py` | Chạy mô hình YOLO `best.onnx` trên ảnh → danh sách khung (tên trạm, độ tin cậy, vị trí) | 🚫 |
| `robot/mission.py` | **"Bộ não" quyết định** – máy trạng thái (giống SFC/GRAFCET): TÌM → TIẾP CẬN → CHỜ 15 s → trạm kế tiếp; vật cản → DỪNG | 🔧 |
| `robot/floor.py` | Nhận diện mặt sàn trống (gạch / bê tông) → biết phía trước còn trống bao xa | 🚫 |
| `robot/navigator.py` | Lách vật cản theo khe trống, nhớ vật cản đã khuất | 🚫 |
| `robot/route_memory.py` | Dạy – lặp lộ trình bằng ảnh mốc (khi trạm bị khuất) | 🚫 |
| `robot/kinematics.py` | Đổi "tiến / ngang / quay" thành tốc độ 4 bánh mecanum | 🚫 |
| `robot/serial_link.py` | Gửi / nhận dòng chữ với Arduino qua USB | 🚫 |
| `robot/display.py` | Màn hình tiếng Việt ("ĐANG DI CHUYỂN ĐẾN TRẠM 2", đếm ngược…) | 🔧 (đổi giao diện) |
| `robot/compass.py`, `odometry.py`, `geometry.py`, `gpio_io.py`, `common.py` | La bàn, đo quãng đường, hình học camera, nút nhấn/còi, kiểu dữ liệu chung | 🚫 |
| `models/best.onnx` + `models/best.metadata.json` | **Mô hình đã train** (dữ liệu, không phải code) – do `export_model.py` tạo | 🚫 |
| `routes/`, `compass_cal.json`, `logs/` | Lộ trình đã dạy, hiệu chuẩn la bàn, nhật ký – tự tạo khi chạy | 🚫 |

### C. Công cụ chạy trên Pi (cần phần cứng robot)

| File | Dùng khi |
|---|---|
| `flash_arduino.py` | nạp Arduino (chạy ở máy đang cắm Arduino – Pi hoặc laptop) |
| `tools/motor_test.py` | thử chiều quay bánh, nhận dạng động cơ, đo đáp ứng SAC-DOB (kê robot lên) |
| `tools/compass_calibrate.py` | hiệu chuẩn la bàn |
| `training/capture_images.py` | chụp ảnh bằng camera robot (đưa nhóm train dùng) |
| `scripts/install_pi.sh` | cài đặt Pi lần đầu |

### D. Chạy trên LAPTOP (không cần robot)

| File | Dùng khi |
|---|---|
| `main.py --sim` (+ `robot/simworld.py`) | chạy thử toàn bộ robot trong mô phỏng |
| `tools/fake_arduino.py` (+ `robot/fake_arduino.py`) | Arduino giả để thử giao tiếp |
| `training/export_model.py` | đổi `best.pt` (nhóm train gửi) → `best.onnx` + metadata (cần `ultralytics`) |
| `training/check_dataset.py` | kiểm tra bộ dữ liệu nhóm train gửi |
| `training/test_model.py` | đánh giá mô hình bằng code robot (chạy được cả trên Pi) |
| `tools/sac_dob_sim.py` | mô phỏng thuật toán động cơ, vẽ đồ thị báo cáo |
| `tests/` | kiểm thử tự động: `python -m pytest -q` |
| `examples/` | **4 bài tập cho người mới** (mục 5) |

### E. Của nhóm train (bạn không cần chạy): `training/train_yolo.py`, `training/split_dataset.py`, `training/data.yaml`

---

## 3. Một khung hình đi qua code như thế nào

Tất cả nằm trong hàm `RobotApp.step()` của `main.py`, lặp khoảng 10 lần mỗi giây:

| # | Ở đâu | Việc | Dữ liệu ra |
|---|---|---|---|
| 1 | `camera.py` → `Camera.read()` | lấy ảnh mới nhất | mảng số `(480, 640, 3)` màu B-G-R |
| 2 | `detector.py` → `detect(frame)` | thu ảnh về 416×416 (đệm viền xám), chạy ONNX, lọc khung trùng | `[Detection("tram_2", 0.91, x1, y1, x2, y2)]` |
| 3 | `floor.py` → `process(frame)` | tìm vùng sàn trống | khoảng trống theo từng hướng (m) |
| 4 | `mission.py` → `update(...)` | quyết định theo trạng thái hiện tại | `vx` (tiến), `wz` (quay), chữ hiển thị |
| 5 | `kinematics.py` → `inverse(vx, vy, wz)` | đổi ra tốc độ 4 bánh | `[30.0, 30.0, 30.0, 30.0]` RPM |
| 6 | `serial_link.py` → `send_wheels()` | gửi qua USB | dòng chữ `W,30.0,30.0,30.0,30.0` |
| 7 | **Arduino** `robot_firmware.ino` → `handleLine()` → `controlStep()` | SAC-DOB tính PWM 50 lần/s | xung PWM → L298N → động cơ |
| 8 | **Arduino** → `sendTelemetry()` → Pi `serial_link.py` | gửi tốc độ đo, pin | `T,...` → `display.py` hiện lên màn hình |

---

## 4. Xử lý ảnh – những từ cần biết

| Thuật ngữ | Nghĩa | Trong dự án |
|---|---|---|
| **Điểm ảnh (pixel)** | một chấm màu của ảnh | ảnh 640×480 = 307 200 điểm |
| **BGR** | mỗi điểm 3 số Xanh dương – Xanh lá – Đỏ (0..255); OpenCV dùng thứ tự B,G,R | `frame[240, 320]` = màu điểm giữa |
| **Khung hình (frame)** | 1 ảnh của video, trong Python là mảng `numpy` | `frame.shape == (480, 640, 3)` |
| **FPS** | số khung xử lý mỗi giây | Pi 4 mục tiêu ≥ 4 FPS |
| **YOLO** | mạng nơ-ron "nhìn 1 lần" – tìm và gọi tên vật trong ảnh | YOLO11n (bản nhỏ nhất, hợp Pi) |
| **Lớp (class)** | loại vật mô hình biết | `0=tram_1, 1=tram_2, 2=tram_3` |
| **Khung (bounding box)** | hình chữ nhật bao quanh vật | `x1, y1, x2, y2` |
| **Tọa độ chuẩn hóa** | tọa độ chia cho kích thước ảnh → 0..1 | `x1 = 0.25` = 25 % bề ngang |
| **Độ tin cậy (confidence)** | mô hình chắc chắn bao nhiêu (0..1) | dưới `STATION_CONF = 0.55` thì bỏ qua |
| **NMS** | bỏ các khung trùng nhau, giữ khung tin cậy nhất | trong `detector.py` |
| **imgsz 416 / letterbox** | mô hình nhận ảnh vuông 416×416; ảnh 4:3 được thu nhỏ và đệm viền xám | tự làm trong `detector.py` |
| **Train** | dạy mô hình bằng ảnh đã gán nhãn → file `best.pt` | việc của nhóm train |
| **ONNX** | định dạng mô hình dùng chung, chạy nhanh trên CPU bằng `onnxruntime`, không cần cài torch | `models/best.onnx` |
| **train / val / test** | ảnh để học / để chọn mô hình / để chấm điểm cuối (không được dùng khi học) | chia theo buổi chụp |
| **recall / precision** | tỉ lệ không bỏ sót / tỉ lệ báo đúng | bảng của `test_model.py --eval` |
| **Ước lượng khoảng cách** | vật càng gần trông càng cao: `d ≈ f × chiều_cao_thật / chiều_cao_khung` | dừng ở 0.45 m trước khối số |

---

## 5. Bốn bài tập – chạy theo thứ tự (thư mục `examples/`)

Mỗi bài 50–80 dòng, chú thích tiếng Việt từng bước. **Mở file đọc trước, rồi mới chạy.**

### Bài 1 – Mở camera, hiểu "ảnh là mảng số" (`01_xem_camera.py`)
```bash
python examples/01_xem_camera.py                 # camera / webcam, ESC để thoát
python examples/01_xem_camera.py --video thu.mp4
```
Kết quả mẫu:
```
khung 11: kích thước (480, 640, 3), điểm giữa B=201 G=214 R=223, 24.3 FPS
```
Thử: đưa vật màu đỏ vào giữa camera → số **R** tăng.

### Bài 2 – Nhận diện khối số trên 1 ảnh (`02_nhan_dien_anh.py`)
Cần `models/best.onnx` + `best.metadata.json` (bước 7 của kế hoạch).
```bash
python examples/02_nhan_dien_anh.py anh_thu.jpg
```
```
Tìm thấy 2 khung (ảnh 640x480):
  tram_1  tin cậy 0.95  khung (519,77)-(625,181)  ~ cách 1.02 m
Đã lưu ảnh kết quả: bai2_ket_qua.jpg
```
Thử: đổi `--conf 0.3` / `--conf 0.8` → thấy số khung thay đổi (ngưỡng tin cậy).

### Bài 3 – Pi gửi lệnh cho Arduino (`03_gui_lenh_arduino.py`)
```bash
python examples/03_gui_lenh_arduino.py --fake                # Arduino giả (Linux / Pi / macOS)
python examples/03_gui_lenh_arduino.py --port /dev/ttyACM0   # Arduino thật – KÊ ROBOT LÊN
```
```
Gửi W,30,... trong 2 giây (gửi 20 lần/giây):
  tốc độ đo được (RPM): [30, 30, 30, 30] | pin: 12.3 V
NGỪNG gửi lệnh 1 giây -> Arduino phải tự dừng:
  tốc độ: [0, 0, 0, 0] | cờ mất lệnh: True
```
Bài này cho thấy cơ chế an toàn: Pi treo → Arduino tự dừng động cơ.

### Bài 4 – "Bộ não" máy trạng thái (`04_may_trang_thai.py`) – không cần phần cứng
```bash
python examples/04_may_trang_thai.py
```
```
t=  0.1s | YOLO: không thấy gì                -> SEARCH   | ĐANG DI CHUYỂN ĐẾN TRẠM 1
t=  0.8s | YOLO: thấy tram_1 ở xa, lệch phải  -> APPROACH | ĐANG DI CHUYỂN ĐẾN TRẠM 1
          lệnh: tiến vx=+0.07 m/s, quay wz=-0.40 rad/s   (lệch phải -> quay phải)
t=  2.3s | YOLO: tram_1 sát trước mặt         -> WAIT     | ĐÃ ĐẾN TRẠM 1
t= 17.4s |                                    -> SEARCH   | ĐANG DI CHUYỂN ĐẾN TRẠM 2
```
Thử: sửa `wait_s` trong `robot/config.py` thành 20 → chạy lại → thời gian chờ đổi theo.

Sau 4 bài, chạy cả robot trong mô phỏng: `python main.py --sim` (phím **O** đặt người chắn đường).

---

## 6. Cài đặt từng thiết bị (làm một lần)

### 6.1 Laptop (Windows)
1. Cài **Python 3.11** từ python.org (tích ô **Add python.exe to PATH**), cài **VS Code** + tiện ích *Python*.
2. Giải nén `robot_xuong_may.zip` → VS Code: *File → Open Folder* → chọn `robot_xuong_may`.
3. Mở *Terminal* trong VS Code (PowerShell) và chạy lần lượt:
   ```powershell
   python -m venv .venv
   .venv\Scripts\activate
   pip install -r requirements.txt
   pip install ultralytics onnx onnxslim
   python -m pytest -q
   python main.py --sim
   ```
   Dòng `pip install ultralytics onnx onnxslim` chỉ để xuất ONNX ở bước 7 (không dùng để train).
   Nếu `activate` báo *running scripts is disabled*: chạy một lần `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.

### 6.2 Raspberry Pi 4 (máy tính nhúng)
1. Trên laptop, dùng **Raspberry Pi Imager** ghi thẻ nhớ: *Raspberry Pi OS (64-bit)*. Bấm ⚙: đặt tên máy `robot`,
   tài khoản `pi` + mật khẩu, Wi-Fi, **bật SSH**.
2. **Tắt nguồn**, cắm cáp dẹt Camera Module 2 vào cổng CSI (mặt kim loại quay về phía cổng HDMI), cắm thẻ, bật nguồn.
3. Từ laptop: `ssh pi@robot.local` – hoặc tiện ích **Remote - SSH** của VS Code để sửa file trên Pi ngay từ laptop.
4. Chép dự án lên Pi và cài:
   ```bash
   # trên laptop
   scp robot_xuong_may.zip pi@robot.local:~
   # trên Pi
   unzip robot_xuong_may.zip && cd robot_xuong_may
   bash scripts/install_pi.sh          # cài thư viện, bật I2C, cài arduino-cli (cần Internet, ~15 phút)
   exit                                # đăng xuất rồi ssh vào lại để nhận quyền mới
   ```
5. Kiểm tra trên Pi:
   ```bash
   cd robot_xuong_may && source .venv/bin/activate
   rpicam-hello -t 3000                          # camera có hình (cần màn hình gắn Pi)
   python examples/01_xem_camera.py --frames 30  # in kích thước ảnh, lưu bai1_khung_cuoi.jpg
   ```

### 6.3 Arduino UNO
1. Đấu dây theo `docs/WIRING.md`. **Tắt nguồn động cơ** khi nạp chương trình.
2. Cắm cáp USB Arduino vào Pi → kiểm tra cổng: `ls /dev/ttyACM* /dev/ttyUSB*`.
3. Trên Pi: `python flash_arduino.py` → phải thấy **"KHỚP cấu hình – nạp thành công!"**.
4. Kê robot lên, bật nguồn động cơ: `python examples/03_gui_lenh_arduino.py --port /dev/ttyACM0`.

---

## 7. Làm việc hằng ngày – sửa cái gì thì làm gì

| Bạn sửa | Phải làm thêm |
|---|---|
| `robot/config.py` nhóm [A], [D] … [K] (camera, trạm, thời gian chờ, ngưỡng…) | chạy lại `python main.py` |
| `robot/config.py` nhóm **[B] Arduino**, **[C] SAC-DOB** | `python flash_arduino.py` (nạp lại Arduino) |
| `firmware/robot_firmware/robot_firmware.ino` | `python flash_arduino.py` |
| `robot/mission.py`, `display.py`… | `python -m pytest -q` rồi chạy lại `main.py` |
| Nhận mô hình mới từ nhóm train | `training/export_model.py` → chép thư mục `models/` sang Pi |

Quy trình gợi ý: **sửa trên laptop → `pytest` → `main.py --sim` → chép sang Pi → chạy thật.**
Dùng VS Code Remote-SSH thì sửa thẳng trên Pi, không cần chép.

---

## 8. Lỗi người mới hay gặp

| Lỗi | Cách tránh |
|---|---|
| `No module named robot` / không tìm thấy file | luôn `cd robot_xuong_may` trước khi chạy lệnh |
| `No module named cv2` / `onnxruntime` | quên kích hoạt môi trường: `source .venv/bin/activate` (Pi) hoặc `.venv\Scripts\activate` (Windows) |
| Windows: `running scripts is disabled on this system` | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` rồi kích hoạt lại |
| `could not open port` khi nạp Arduino | đang chạy `main.py` hoặc `motor_test.py` (cổng bận) → tắt trước |
| Robot báo "Firmware Arduino khác config.py" | sửa nhóm [B]/[C] mà chưa `flash_arduino.py` |
| Sửa `config_gen.h` bằng tay rồi bị mất | file này tự sinh – sửa `robot/config.py` |
| Nạp bằng Arduino IDE rồi robot chạy sai | luôn nạp bằng `python flash_arduino.py` |
| Camera Pi chết sau khi rút cáp | **không** rút / cắm cáp dẹt CSI khi Pi đang bật |
| Windows: `tools/fake_arduino.py` lỗi | Windows không có cổng ảo → dùng `python main.py --fake-link` |
| Pi khởi động lại / chậm bất thường | nguồn 5 V yếu (cần ≥ 3 A) hoặc CPU quá nóng (gắn tản nhiệt + quạt) |

---

**Đọc tiếp:** `docs/HUONG_DAN_NGAY_1.md` (việc của bạn theo kế hoạch ngày 1) · `docs/WIRING.md` (đấu dây) ·
`README.md` (tổng quan, chạy thử từng bước) · `docs/SAC_DOB.md` (thuật toán động cơ) · `docs/NAVIGATION.md` (dẫn đường camera).
