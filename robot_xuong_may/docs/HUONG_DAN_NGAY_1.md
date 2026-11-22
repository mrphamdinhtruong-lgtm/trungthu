# HƯỚNG DẪN CHI TIẾT CHO NGƯỜI VIẾT CODE – KẾ HOẠCH TRAIN 3 TRẠM (NGÀY 1)

Bám theo file kế hoạch **`Ke_hoach_train_3_tram_1_ngay_1.xlsx`** (08:00 → 18:00, 10 bước).
Bạn **chỉ viết code**; 3 người còn lại chụp, gán nhãn và train. Code nền đã có sẵn trong `robot_xuong_may/`
– việc của bạn trong ngày là **chạy, kiểm tra, tích hợp, sửa** và **nhận / bàn giao đúng định dạng** với nhóm train.

> Bản Excel đi kèm (cùng 10 bước, thêm cột lệnh cụ thể + bảng bàn giao):
> `docs/Ke_hoach_train_3_tram_1_ngay_1_HUONG_DAN_CODE.xlsx`

---

## 0. Những gì đã CHỐT theo kế hoạch (code đã đặt sẵn)

| Hạng mục | Giá trị | Ở đâu trong code |
|---|---|---|
| Lớp & thứ tự | `0 = tram_1`, `1 = tram_2`, `2 = tram_3` | `robot/config.py` → `MODEL_CLASSES`, `STATIONS[n]["class"]`; `training/data.yaml` |
| Mô hình | 1 mô hình **YOLO11n Detect** chung 3 trạm | `training/train_yolo.py` (mặc định `yolo11n.pt`) |
| Train | **50 epochs, ảnh 416, batch Auto** | `training/train_yolo.py` (mặc định 50 / 416 / -1) |
| Mô hình chạy trên robot | `models/best.onnx` + `models/best.metadata.json` | `robot/config.py` → `MODEL_PATH`; `training/export_model.py` |
| Chờ tại trạm | **15 giây**, có đếm ngược | `STATIONS[n]["wait_s"] = 15.0` |
| Chia dữ liệu | train / val / test **theo buổi chụp** | `training/split_dataset.py` |
| Vật cản | mô hình 3 lớp **không có** lớp vật cản → dùng **nhận diện sàn** | `robot/floor.py` (`FLOOR_ENABLE = True`) |

> **Lưu ý 15 s / 20 s:** yêu cầu đề tài ban đầu là 20 s, kế hoạch ngày 1 ghi 15 s → code đang để 15 s.
> Muốn đổi: sửa 3 số `wait_s` trong `robot/config.py` (không cần nạp lại Arduino).

> **"Code V4" trong kế hoạch:** tôi không có bản code V4 nào khác. Script `training/export_model.py`
> hiện tạo đúng 2 file kế hoạch yêu cầu (`best.onnx`, `best.metadata.json`). Nếu V4 của bạn dùng định dạng
> metadata khác, gửi lại để tôi chỉnh cho khớp.

### Bản đồ: bước kế hoạch ↔ file code

| Bước | Việc của bạn (kế hoạch) | File / lệnh chính |
|---|---|---|
| 1 | Chốt lớp, giao diện nhận diện, cấu trúc | `robot/config.py`, `robot/detector.py`, `training/data.yaml` |
| 2 | Đọc ảnh / video / camera, xử lý mất hình | `robot/camera.py`, `main.py --images / --video` |
| 3 | Giao tiếp Pi–Arduino, timeout, lệnh dừng, dữ liệu giả | `robot/serial_link.py`, `tools/fake_arduino.py`, `firmware/` |
| 4 | Sao lưu code | `git commit`, `git archive` |
| 5 | Kiểm tra lớp, định dạng file, dữ liệu đầu vào | `training/split_dataset.py` |
| 6 | Logic 3 trạm, chờ 15 s, đếm ngược, mất dữ liệu | `robot/mission.py`, `main.py --sim` |
| 7 | Nhận .pt, kiểm tra lớp, xuất ONNX, tích hợp | `training/export_model.py`, `robot/detector.py` |
| 8 | Chạy bằng ảnh/video nhóm gửi, kiểm tra hiển thị | `training/test_model.py --eval`, `main.py --video` |
| 9 | Sửa lỗi code/tích hợp, nhận mô hình mới | bảng phân loại lỗi §9 |
| 10 | Đóng gói, hướng dẫn chạy, ghi phiên bản mô hình | `git`, `models/best.metadata.json` |

---

## Chuẩn bị (tối hôm trước hoặc 07:30–08:00)

**Laptop của bạn** (Windows / Linux / macOS, Python 3.10–3.12):
```bash
cd robot_xuong_may
python -m venv .venv
# Windows:  .venv\Scripts\activate        Linux/macOS:  source .venv/bin/activate
pip install -r requirements.txt -r requirements-train.txt
python -m pytest -q                 # phải thấy: 103 passed
python main.py --sim                # cửa sổ giao diện: robot đi TRẠM 1 → 2 → 3, đếm ngược 15 s
```
**Raspberry Pi 4** (có Internet):
```bash
bash scripts/install_pi.sh          # apt + venv + thư viện + arduino-cli + kiểm thử
rpicam-hello -t 3000                # camera CSI có hình
```
**Thống nhất với nhóm:** một thư mục chia sẻ (Drive / Zalo / USB) với 3 thư mục con `anh_goc/`, `nhan/`, `mo_hinh/`.

### Hợp đồng bàn giao (gửi nhóm train ngay 08:00)

| Thứ cần nhận | Định dạng bắt buộc | Ai gửi | Khi nào |
|---|---|---|---|
| Ảnh gốc | `.jpg`, tỉ lệ **4:3**, ≥ 640×480; **mỗi buổi chụp 1 thư mục** (vd `A_sang/`, `B_chieu/`) | Người 1 | theo đợt từ 08:30 |
| Nhãn | YOLO `.txt` cùng tên ảnh, mỗi dòng `lớp cx cy w h` (0..1), lớp **0/1/2** | Người 2, 3 | đến 13:00 |
| `data.yaml` | `names: {0: tram_1, 1: tram_2, 2: tram_3}` đúng thứ tự | Người 3 | 13:00 |
| Mô hình | `best.pt` + `args.yaml` + `results.csv` (+ `confusion_matrix.png`) | Người 3 | ~14:30 |
| Phiên bản | chuỗi `tram3_v1`, `tram3_v2`… (tăng mỗi lần train lại) | Người 3 | cùng `.pt` |
| Ảnh/video thử | cảnh MỚI (không có trong train), quay ngang 4:3 nếu được | Người 1, 2 | 14:30 |

---

## Bước 1 · 08:00–08:30 · Chốt lớp, giao diện nhận diện, cấu trúc chương trình

**Mục tiêu:** cả nhóm dùng đúng 3 tên lớp, đúng thứ tự; bạn biết rõ code cần nhận gì.

1. Mở `robot/config.py`, kiểm tra:
   ```python
   MODEL_CLASSES = ("tram_1", "tram_2", "tram_3")          # 0, 1, 2
   STATIONS = {1: {"class": "tram_1", ... "wait_s": 15.0, "marker_h_m": 0.20, ...}, ...}
   ```
   và `training/data.yaml` (`names: 0: tram_1, 1: tram_2, 2: tram_3`). Gửi ảnh chụp 2 chỗ này cho nhóm.
2. **Giao diện nhận diện** (nói cho nhóm hiểu code dùng mô hình thế nào):
   ```python
   from robot.detector import create_detector
   det = create_detector("models/best.onnx")       # đọc best.metadata.json -> tên lớp, ảnh 416
   dets = det.detect(frame_bgr)                    # ảnh BGR (OpenCV) bất kỳ cỡ
   # -> [Detection(cls="tram_2", conf=0.91, x1, y1, x2, y2)]   tọa độ chuẩn hóa 0..1
   ```
   Máy trạng thái so `cls` với `STATIONS[n]["class"]` → tên lớp sai một ký tự là robot **không bao giờ thấy trạm**.
3. **Đo khối số** (thước): chiều cao khối (m) → `marker_h_m` của từng trạm. Đây là cơ sở ước lượng khoảng cách & chỗ dừng.
4. Gửi nhóm bảng "Hợp đồng bàn giao" ở trên + cấu hình train: `YOLO11n Detect, 50 epochs, imgsz 416, batch Auto`.
5. Nhắc Người 1: ngoài camera ngoài, **chụp thêm bằng camera robot** (Pi) ở đúng độ cao:
   `python training/capture_images.py --session robot_sang` → giảm lệch giữa ảnh train và ảnh thật.

**Đạt khi:** nhóm xác nhận `0=tram_1, 1=tram_2, 2=tram_3`; `pytest` 103 passed; `main.py --sim` chạy.

---

## Bước 2 · 08:30–10:30 · Đọc ảnh, video, camera; xử lý khi mất hình

**Mục tiêu:** chương trình chạy được với **3 nguồn hình** và **dừng an toàn khi mất hình**.

| Nguồn | Lệnh (chưa có mô hình vẫn chạy – chỉ hiện cảnh báo) |
|---|---|
| Thư mục ảnh | `python main.py --fake-link --images anh_thu/ --fps 2 --no-floor` |
| Video | `python main.py --fake-link --video thu.mp4 --no-floor` |
| Webcam laptop | `python main.py --fake-link` (`CAMERA_BACKEND = "auto"`: không có picamera2 thì tự dùng webcam) |
| Camera robot (Pi) | `python main.py --fake-link` (mặc định `picamera2`) |

`--fake-link` = Arduino giả trong chương trình (robot không chạy). `--no-floor` = tắt nhận diện sàn khi ảnh không quay từ camera robot.

**Thử mất hình** (dùng webcam USB, **không bao giờ rút cáp dẹt CSI khi Pi đang bật**):
1. Chạy `python main.py --fake-link`, nhấn **Enter** (chạy tự động).
2. Rút webcam → trong ≤ 2 s (`CAMERA_TIMEOUT_S`) màn hình đỏ **"LỖI HỆ THỐNG – ROBOT DỪNG / Lỗi camera"**, lệnh xuống Arduino là `S` (dừng).
3. Cắm lại → log `Mất hình -> mở lại camera`, màn hình về **"SẴN SÀNG – Đã khắc phục lỗi – nhấn START"** (robot **không tự chạy lại** – an toàn).

Kiểm tra tự động: `python -m pytest tests/test_link_camera_dataset.py -k camera -v` và `tests/test_mission.py -k fault`.

**Đạt khi:** 3 nguồn hình đều lên giao diện; mất hình → dừng + báo lỗi; có hình lại → chờ START.

---

## Bước 3 · 10:30–12:00 · Giao tiếp Pi–Arduino, timeout, lệnh dừng – thử bằng dữ liệu giả

**Mục tiêu:** kiểm chứng giao thức, timeout, dừng khẩn **trước khi** có Arduino thật.

Giao thức (USB-Serial 115200, ASCII, xem đầu `firmware/robot_firmware/robot_firmware.ino`):

| Pi → Arduino | Ý nghĩa | Arduino → Pi | Ý nghĩa |
|---|---|---|---|
| `W,fl,fr,rl,rr` | tốc độ 4 bánh (RPM) – gửi đều ~10–20 Hz = "còn sống" | `I,ROBOT_FW,ver,hash,...` | thông tin + mã băm cấu hình |
| `S` | dừng | `T,ms,rpm×4,u×4,cờ,mode,vbat` | telemetry 20 Hz; cờ bit0 dừng khẩn, bit1 mất lệnh |
| `E,1` / `E,0` | dừng khẩn / xóa | `A,OK,...` / `A,ERR,...` | phản hồi |

**A. Arduino giả qua cổng serial ảo** (Linux / Pi / macOS; Windows dùng `--fake-link`):
```bash
# cửa sổ 1
python tools/fake_arduino.py --verbose          # in: Arduino giả lập đang chạy ở cổng: /dev/pts/5
# cửa sổ 2
python main.py --port /dev/pts/5 --images anh_thu/ --no-floor
```
| Thử | Lệnh cửa sổ 1 | Kết quả đúng trên màn hình |
|---|---|---|
| Kết nối | (mặc định) | không có cảnh báo "Firmware khác config"; RPM hiện ở bảng phải |
| Mất cáp | `--drop-after 20` | sau ~21 s: **"Mất kết nối Arduino (kiểm tra cáp USB)"**, robot dừng |
| Dừng khẩn | `--estop-at 15` | **"DỪNG KHẨN CẤP"**; nhấn **R** để reset |
| Timeout lệnh | tắt `main.py` | Arduino giả tự đưa RPM về 0 sau 0.5 s (cờ bit1) |

Kiểm tra tự động: `python -m pytest tests/test_link_camera_dataset.py -k serial -v`.

**B. Arduino thật** (nếu có): kê robot lên rồi
```bash
python flash_arduino.py --install-cli      # lần đầu; đạt khi: "KHỚP cấu hình – nạp thành công!"
python tools/motor_test.py params          # đọc tham số SAC-DOB
python tools/motor_test.py dir             # từng bánh quay tiến, đo được RPM
```
Rút USB khi bánh đang quay → bánh phải dừng trong 0.5 s (`CMD_TIMEOUT_MS`).

**Đạt khi:** 4 thử nghiệm bảng A đúng; (nếu có Arduino) nạp khớp mã băm, timeout dừng bánh.

---

## Bước 4 · 12:00–12:30 · Nghỉ & sao lưu code

```bash
git add -A && git commit -m "Ngày 1 – trưa: camera, giao tiếp Pi-Arduino đã kiểm tra"
git archive --format=zip -o ../robot_xuong_may_trua.zip HEAD    # chép file zip vào USB / Drive
```
(Đẩy GitHub cần cấp quyền Claude GitHub App cho repo – xem README.)

---

## Bước 5 · 12:30–13:00 · Kiểm tra tên/thứ tự lớp, định dạng file, dữ liệu đầu vào

**Mục tiêu:** bộ dữ liệu gộp của 3 người đúng định dạng, đủ 3 lớp, không rò ảnh gần trùng sang tập test.

1. Nhận bộ nhãn gộp (Người 3), đặt theo buổi: `dataset/labeled/<buổi>/*.jpg + *.txt`.
2. Chạy:
   ```bash
   python training/split_dataset.py --src dataset/labeled --val 0.2 --test 0.1
   ```
   Đọc báo cáo:
   | Dòng báo | Ý nghĩa | Xử lý |
   |---|---|---|
   | bảng `tram_1 tram_2 tram_3` mỗi tập | số khối mỗi lớp | lớp nào < 50 ở train → chụp thêm |
   | `!! tập test KHÔNG có đối tượng lớp …` | thiếu lớp ở val/test | đổi `--seed` hoặc thêm buổi |
   | `!! … dòng nhãn SAI định dạng / mã lớp` | mã lớp ≥ 3, tọa độ ngoài 0..1 | trả về người gán nhãn sửa |
   | `!! … GẦN TRÙNG ảnh train` | ảnh test trùng ảnh học | chuyển cả buổi sang train / chụp buổi mới |
   | `!! Chỉ có 2 buổi chụp` | không chia theo buổi được | chụp ít nhất 4–6 buổi |
3. Nếu nhóm chia sẵn trên cloud: vẫn kiểm tra `data.yaml` đúng thứ tự lớp và mở ngẫu nhiên 10 file `.txt`.
4. **Dữ liệu đầu vào code cần lúc chạy:** ảnh BGR, code tự đưa về 640×480. Video 16:9 sẽ bị bóp ngang
   → khoảng cách ước lượng vẫn đúng (dựa chiều cao khối) nhưng nên quay 4:3 để giống camera robot.

**Đạt khi:** báo cáo không còn dòng `!!`; cả train/val/test đều có đủ `tram_1`, `tram_2`, `tram_3`.

---

## Bước 6 · 13:00–14:30 · Logic 3 trạm, chờ 15 s, đếm ngược, xử lý mất dữ liệu

**Mục tiêu:** máy trạng thái chạy đúng thứ tự trạm, đếm ngược 15 s, xử lý mọi loại "mất dữ liệu".

1. Thông số trong `robot/config.py`:
   | Thông số | Mặc định | Ý nghĩa |
   |---|---|---|
   | `ROUTE`, `ROUTE_LOOP` | `(1, 2, 3)`, `True` | thứ tự trạm, chạy vòng |
   | `wait_s` | 15.0 | thời gian chờ mỗi trạm |
   | `ARRIVE_DISTANCE_M` | 0.45 | khoảng cách coi là đã đến |
   | `STATION_CONF`, `CONFIRM_FRAMES` | 0.55, 3 | ngưỡng tin cậy, số khung xác nhận |
   | `TARGET_LOST_HOLD_S`, `TARGET_LOST_TIMEOUT_S` | 0.4, 1.5 | mất dấu khối ngắn / lâu |
   | `SEARCH_TIMEOUT_S`, `LOST_AUTO_RETRY_S` | 30, 10 | quá lâu không thấy trạm |
2. Chạy mô phỏng và quan sát: `python main.py --sim` → TRẠM 1 → 2 → 3 → 1, số đếm ngược lớn **15 → 0**,
   phím **O** đặt người chắn đường ("GẶP VẬT CẢN"), **N** bỏ qua thời gian chờ.
3. Kiểm tra tự động: `python -m pytest tests/test_mission.py tests/test_mission_sim.py -v`.
4. Bảng xử lý mất dữ liệu (đã có trong code – kiểm tra lại từng dòng bằng mô phỏng / dữ liệu giả):
   | Mất gì | Robot làm gì | Màn hình |
   |---|---|---|
   | Khối số chập chờn < 0.4 s | giữ lệnh cũ giảm 50 % | "Tạm mất dấu biển báo..." |
   | Mất khối 0.4–1.5 s | đứng chờ | "Mất dấu biển báo – đang chờ..." |
   | Mất khối > 1.5 s | quay tìm về phía thấy lần cuối | "ĐANG DI CHUYỂN ĐẾN TRẠM n – Đang quay tìm..." |
   | Không thấy trạm > 30 s | dừng, tự tìm lại sau 10 s | "KHÔNG TÌM THẤY TRẠM n" |
   | Mất hình > 2 s | dừng | "LỖI HỆ THỐNG – Lỗi camera" |
   | Mất Arduino > 1 s | dừng | "LỖI HỆ THỐNG – Mất kết nối Arduino" |
   | Pi treo / mất lệnh > 0.5 s | Arduino tự dừng động cơ | (cờ timeout) |
5. Trong lúc đó Người 3 train: hỏi **ETA** mỗi 15 phút. ETA > 90 phút → sửa ô **Phút** của bước 6 trong file Excel
   (giờ các bước sau tự tính lại) và báo cả nhóm.

**Đạt khi:** mô phỏng đi đúng thứ tự, đếm ngược đúng 15 s; test mission đều qua; biết ETA train.

---

## Bước 7 · 14:30–15:15 · Nhận .pt → kiểm tra lớp → xuất ONNX → tích hợp

1. Nhận từ Người 3 vào `nhan/`: `best.pt`, `args.yaml`, `results.csv`, tên phiên bản (vd `tram3_v1`).
2. Xuất:
   ```bash
   python training/export_model.py --weights nhan/best.pt --version tram3_v1 \
          --args nhan/args.yaml --results nhan/results.csv
   ```
   Kết quả đúng (ví dụ thật từ lần chạy thử):
   ```
   [export] Lớp OK: {0: 'tram_1', 1: 'tram_2', 2: 'tram_3'}
   [export] Đã tạo models/best.onnx và models/best.metadata.json
   [export] Phiên bản: tram3_v1  |  ảnh 416  |  3 lớp
   [check] best.onnx: phiên bản tram3_v1, lớp {...}, ảnh (416, 416), onnxruntime, 23 ms/khung
   [check] lưu ý: Mô hình không có lớp vật cản -> chỉ phát hiện vật cản bằng nhận diện sàn
   [check] mã băm ONNX khớp metadata
   ```
   Nếu thấy `[export] DỪNG: Tên/thứ tự lớp ... KHÁC config` → **không tự sửa code cho khớp**, trả về Người 3 train lại đúng thứ tự.
3. `models/best.metadata.json` chứa: `model_version`, `names`, `nc`, `imgsz`, mã băm `.pt`/`.onnx`, `train_args`
   (epochs, imgsz, batch...), `metrics` (mAP50, precision, recall). Đây là bằng chứng "đúng phiên bản" khi bàn giao.
4. Chép sang Pi và kiểm tra trên Pi:
   ```bash
   scp -r models pi@<ip-pi>:~/robot_xuong_may/
   ssh pi@<ip-pi> "cd robot_xuong_may && .venv/bin/python training/export_model.py --check models/best.onnx"
   ```
   Ghi lại **ms/khung trên Pi** (mục tiêu ≤ 250 ms ≈ 4 FPS). Chậm hơn → xuất lại `--imgsz 320` (độ chính xác giảm nhẹ).
5. Chạy `python main.py --fake-link` → log `Nạp best.onnx (onnxruntime): lớp {...}, phiên bản tram3_v1`,
   bảng phải màn hình: **"Mô hình: tram3_v1 (onnxruntime)"**. Sai lớp → robot **không cho chạy tự động**.

**Đạt khi:** có `best.onnx` + `best.metadata.json` đúng 3 nhãn; `--check` không có dòng LỖI; Pi chạy được mô hình.

---

## Bước 8 · 15:15–16:45 · Chạy code bằng ảnh/video nhóm gửi; kiểm tra hiển thị, trạng thái, bộ đếm

1. **Đánh giá trên tập test giữ riêng** (cùng code robot):
   ```bash
   python training/test_model.py --eval dataset --split test
   ```
   ```
   lớp        đúng   nhầm  bỏ sót  báo giả  recall precision
   tram_1       11      0       0        0    1.00      1.00
   tram_2        9      0       2        1    0.82      0.90
   ```
   File `logs/eval_test_*.csv` liệt kê từng ảnh **NHẦM TRẠM / BỎ SÓT / BÁO GIẢ** → gửi Người 2 đối chiếu.
   **NHẦM TRẠM là lỗi nguy hiểm nhất** (robot dừng sai trạm) – phải bằng 0.
2. **Video / ảnh cảnh mới:**
   ```bash
   python training/test_model.py --source video_moi.mp4               # khung + khoảng cách ước lượng + FPS
   python main.py --fake-link --video video_moi.mp4 --no-floor        # toàn bộ giao diện robot
   python main.py --fake-link --images anh_moi/ --fps 1 --no-floor
   ```
3. **Camera robot + khối thật** (trên Pi, robot không chạy vì `--fake-link`): đưa khối lại gần dần, đối chiếu:
   | Kiểm tra trên màn hình | Đúng khi |
   |---|---|
   | Khối ở xa, đúng trạm đích | băng xanh **"ĐANG DI CHUYỂN ĐẾN TRẠM n"**, "còn khoảng x m" |
   | Khoảng cách | sai ≤ 10 % so với thước (sai → sửa `marker_h_m`, `CAMERA_HEIGHT_M`) |
   | Khối ≤ 0.45 m, ở giữa ảnh | băng xanh lá **"ĐÃ ĐẾN TRẠM n"**, số lớn đếm **15 → 0** |
   | Hết 15 s | chuyển **"ĐANG DI CHUYỂN ĐẾN TRẠM n+1"**, vòng tròn lộ trình sáng trạm kế |
   | Khối của trạm KHÁC | không phản ứng (vẫn tìm trạm đích) |
   | Che camera | "LỖI HỆ THỐNG – Lỗi camera" sau 2 s |
   | FPS (bảng phải) | ≥ 4 FPS trên Pi |
4. Người 1 quay lại các tình huống nhận sai → nhận về thư mục `loi/` để chạy lại bước 1–2 của mục này.

**Đạt khi:** có bảng đánh giá test + CSV lỗi; giao diện đúng bảng trên với ảnh/video bàn giao và camera thật.

---

## Bước 9 · 16:45–17:45 · Sửa lỗi code/tích hợp; nhận mô hình mới nếu train lại

**Phân biệt lỗi CODE và lỗi DỮ LIỆU:**

| Hiện tượng | Thử | Kết luận | Ai sửa |
|---|---|---|---|
| `test_model.py` cũng không thấy khối | xem `eval_*.csv` | **dữ liệu / mô hình** (thiếu ảnh góc đó) | nhóm chụp bổ sung, train lại |
| `test_model.py` thấy nhưng robot không phản ứng | so tên lớp trong log với `STATIONS` | **code / cấu hình** (`class`, `STATION_CONF`, `CONFIRM_FRAMES`) | bạn |
| Nhầm `tram_2` ↔ `tram_3` | `eval` cột "nhầm" | **dữ liệu** (khối giống nhau, nhãn sai) | sửa nhãn / đổi màu khối |
| Dừng quá xa / quá gần | đo thước | **cấu hình** `marker_h_m`, `ARRIVE_DISTANCE_M` | bạn |
| Chạy được trên laptop, lỗi trên Pi | `--check` trên Pi | **môi trường** (onnxruntime, quyền camera) | bạn |
| Báo vật cản giả khi thử bằng video | thêm `--no-floor` | bình thường (video không quay từ camera robot) | – |
| FPS < 3 trên Pi | `--check` ms/khung | **hiệu năng** → xuất `--imgsz 320`, tản nhiệt | bạn |

**Nhận mô hình mới:**
```bash
python training/export_model.py --weights nhan_v2/best.pt --version tram3_v2 --args nhan_v2/args.yaml --results nhan_v2/results.csv
python training/test_model.py --eval dataset --split test        # so với bảng v1 – giữ bản tốt hơn
```
Không được đổi tập test giữa v1 và v2 (nếu không so sánh vô nghĩa). Mỗi lần xuất, `best.metadata.json` ghi phiên bản mới.

**Đạt khi:** mọi lỗi trong danh sách được xếp loại code / dữ liệu; lỗi code đã sửa hoặc ghi lại.

---

## Bước 10 · 17:45–18:00 · Đóng gói, hướng dẫn chạy, ghi phiên bản mô hình

```bash
python -m pytest -q                                    # vẫn 103 passed (hoặc hơn nếu bạn thêm test)
git add -A && git commit -m "Ngày 1: tích hợp mô hình tram3_vX, đã đánh giá test"
git archive --format=zip -o ../robot_xuong_may_ngay1.zip HEAD
```
Ghi vào biên bản bàn giao: **phiên bản mô hình đang dùng** (`model_version` trong `models/best.metadata.json`),
mã băm `sha256_onnx`, bảng đánh giá test, danh sách lỗi chưa xử lý.

**Chưa làm trong ngày 1 (đúng kế hoạch):** đánh giá tự hành trên xe thật. Việc cho ngày 2:
`flash_arduino.py` → `motor_test.py dir / count / pattern / ident / step` → học màu sàn (phím **F**) →
dạy lộ trình nếu trạm khuất (**M → T**) → chạy tự động có người giám sát (xem README §5).

---

## Phụ lục A – Lệnh nhanh

| Việc | Lệnh |
|---|---|
| Kiểm thử toàn bộ | `python -m pytest -q` |
| Mô phỏng | `python main.py --sim` |
| Chạy bằng ảnh / video | `python main.py --fake-link --images DIR --no-floor` · `--video FILE` |
| Arduino giả qua cổng ảo | `python tools/fake_arduino.py [--drop-after 20] [--estop-at 15]` |
| Chia dữ liệu theo buổi | `python training/split_dataset.py --src dataset/labeled --val 0.2 --test 0.1` |
| Train (máy có GPU) | `python training/train_yolo.py` |
| Xuất ONNX + metadata | `python training/export_model.py --weights best.pt --version tram3_v1` |
| Kiểm tra ONNX | `python training/export_model.py --check models/best.onnx` |
| Đánh giá test | `python training/test_model.py --eval dataset --split test` |
| Nạp Arduino | `python flash_arduino.py` |

## Phụ lục B – Lỗi hay gặp

| Thông báo | Nguyên nhân | Cách xử lý |
|---|---|---|
| `Chưa có mô hình YOLO – chỉ chạy được chế độ tay` | thiếu `models/best.onnx` hoặc `.metadata.json` | bước 7 |
| `Thiếu tên lớp: cần file best.metadata.json` | chép thiếu file | chép cả thư mục `models/` |
| `Mô hình KHÔNG có lớp 'tram_1'` / `Thứ tự lớp sai` | nhóm đặt tên khác (`tram1`, `Tram 1`...) | train lại đúng tên – không sửa code |
| `No module named onnxruntime` | thiếu thư viện | `pip install onnxruntime` |
| `No module named picamera2` trong venv | venv tạo thiếu `--system-site-packages` | tạo lại venv theo `install_pi.sh` |
| `Không tìm thấy Arduino` / `Permission denied /dev/ttyACM0` | chưa cắm / thiếu quyền | `sudo usermod -aG dialout $USER` rồi đăng nhập lại |
| `Firmware Arduino khác config.py` | sửa config chưa nạp lại | `python flash_arduino.py` |
| `tools/fake_arduino.py` lỗi trên Windows | Windows không có pty | dùng `--fake-link` |
| `simplifier failure: No module named 'onnxslim'` khi xuất | thiếu gói phụ | `pip install onnxslim` (không bắt buộc) |

## Phụ lục C – Rủi ro trong ngày và cách dự phòng

| Rủi ro | Dấu hiệu | Dự phòng |
|---|---|---|
| Ảnh camera ngoài khác camera robot | test tốt, camera robot kém | chụp bổ sung bằng camera robot (bước 1.5), train lại ở bước 9 |
| Train lâu hơn dự kiến | ETA > 90 phút | giảm epochs không dưới 30, hoặc lùi bước 7–8 (sửa ô Phút trong Excel) |
| Pi chạy chậm ở 416 | > 250 ms/khung | xuất lại `--imgsz 320`; gắn tản nhiệt / quạt |
| Mất dữ liệu nhãn | – | sao lưu bước 4 + mỗi khi nhận đợt nhãn mới |
| Nhầm phiên bản mô hình | metadata khác tên | luôn xuất bằng `--version`, đọc dòng "Mô hình:" trên màn hình |
