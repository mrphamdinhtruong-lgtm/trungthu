# HƯỚNG DẪN CHO NGƯỜI VIẾT CODE – KẾ HOẠCH TRAIN 3 TRẠM (NGÀY 1)

Bám theo file **`Ke_hoach_train_3_tram_1_ngay_1.xlsx`** (08:00 → 18:00, 10 bước).
**Bạn chỉ viết code. Ba người còn lại chụp ảnh, gán nhãn, gộp dữ liệu và train.**
Ở những chỗ hai bên gặp nhau, bạn chỉ **nhận – kiểm tra – tích hợp**, không làm thay phần train.

> Bản Excel đi kèm: `docs/Ke_hoach_train_3_tram_1_ngay_1_HUONG_DAN_CODE.xlsx`
> (sheet gốc giữ nguyên + "Huong dan code" + "Ban giao" + "Gui nhom train").
> Trang gửi cho 3 người train: **`docs/YEU_CAU_NHOM_TRAIN.md`**.

---

## 0. Phân vai (đúng theo kế hoạch)

| Công việc | **Bạn (code)** | Người train 1 | Người train 2 | Người train 3 |
|---|---|---|---|---|
| Chụp ảnh khối số, quay video thử | – | ✔ | | |
| Gán nhãn | – | phần A | phần B | phần C |
| Gộp nhãn, chia train / val / test theo buổi | **kiểm tra** | | | ✔ |
| Train YOLO11n (cloud) | – | | | ✔ |
| Kiểm tra mô hình trên tập test | **đối chiếu bằng code robot** | | ✔ | |
| Xuất `best.onnx` + `best.metadata.json` | ✔ | | | |
| Code camera / video / ảnh, giao tiếp Pi–Arduino, logic 3 trạm, hiển thị | ✔ | | | |
| Đóng gói code, ghi phiên bản mô hình | ✔ | | | |

### Đã chốt (code đặt sẵn)

| Hạng mục | Giá trị | Ở đâu |
|---|---|---|
| Lớp & thứ tự | `0 = tram_1`, `1 = tram_2`, `2 = tram_3` | `robot/config.py` → `MODEL_CLASSES`, `STATIONS[n]["class"]` |
| Mô hình robot dùng | `models/best.onnx` + `models/best.metadata.json` | `MODEL_PATH`; tạo bằng `training/export_model.py` |
| Chờ tại trạm | 15 giây, có đếm ngược | `STATIONS[n]["wait_s"]` (đề tài gốc 20 s → sửa 3 số này nếu cần) |
| Vật cản | mô hình 3 lớp không có lớp vật cản → **nhận diện sàn** | `FLOOR_ENABLE = True` |

> **"Code V4" trong kế hoạch:** tôi không có bản V4 nào khác; `training/export_model.py` tạo đúng 2 file kế hoạch yêu cầu.
> Nếu V4 của bạn dùng định dạng metadata khác, gửi để tôi chỉnh cho khớp.

### Code của bạn nằm ở đâu

| Bước | Việc của bạn | File / lệnh |
|---|---|---|
| 1 | Chốt lớp, giao diện nhận diện, cấu trúc | `robot/config.py`, `robot/detector.py` |
| 2 | Đọc ảnh / video / camera, xử lý mất hình | `robot/camera.py`, `main.py --images / --video` |
| 3 | Giao tiếp Pi–Arduino, timeout, lệnh dừng | `robot/serial_link.py`, `tools/fake_arduino.py`, `firmware/` |
| 4 | Sao lưu code | `git` |
| 5 | Kiểm tra dữ liệu đầu vào nhóm gửi | `training/check_dataset.py` |
| 6 | Logic 3 trạm, chờ 15 s, đếm ngược, mất dữ liệu | `robot/mission.py`, `main.py --sim` |
| 7 | Nhận `.pt`, kiểm tra lớp, xuất ONNX, tích hợp | `training/export_model.py` |
| 8 | Chạy code bằng ảnh / video nhóm gửi | `training/test_model.py --eval`, `main.py --video` |
| 9 | Sửa lỗi code / tích hợp, nhận mô hình mới | bảng phân loại lỗi §9 |
| 10 | Đóng gói, hướng dẫn chạy, ghi phiên bản | `git`, `models/best.metadata.json` |

---

## Chuẩn bị (tối hôm trước hoặc 07:30–08:00)

**Laptop của bạn:**
```bash
cd robot_xuong_may
python -m venv .venv
# Windows: .venv\Scripts\activate      Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
pip install ultralytics onnx onnxslim   # CHỈ để chuyển best.pt -> best.onnx ở bước 7 (không dùng để train)
python -m pytest -q                     # phải thấy: 113 passed
python main.py --sim                    # robot mô phỏng đi TRẠM 1 → 2 → 3, đếm ngược 15 s
```
**Raspberry Pi 4:** `bash scripts/install_pi.sh` rồi `rpicam-hello -t 3000` (camera có hình).

**08:00 gửi ngay cho 3 người train:** file `docs/YEU_CAU_NHOM_TRAIN.md` (tên lớp, định dạng nhãn, thư mục theo buổi,
những gì phải bàn giao cho bạn và lúc nào).

---

## Bước 1 · 08:00–08:30 · Chốt tên lớp, giao diện code nhận diện, cấu trúc chương trình

1. Mở `robot/config.py`, xác nhận:
   ```python
   MODEL_CLASSES = ("tram_1", "tram_2", "tram_3")        # mã 0, 1, 2
   STATIONS = {1: {"class": "tram_1", "wait_s": 15.0, "marker_h_m": 0.20, ...}, ...}
   ```
   Đọc to cho Người 3 nghe để họ tạo dataset cloud đúng **tên và thứ tự** (`tram_1` có gạch dưới, chữ thường).
2. **Giao diện nhận diện** – phần code của bạn sẽ dùng mô hình như sau:
   ```python
   from robot.detector import create_detector
   det = create_detector("models/best.onnx")    # đọc best.metadata.json -> tên lớp, ảnh 416
   dets = det.detect(frame_bgr)                 # ảnh BGR của OpenCV, cỡ bất kỳ
   # -> [Detection(cls="tram_2", conf=0.91, x1=..., y1=..., x2=..., y2=...)]   tọa độ 0..1
   ```
   Máy trạng thái so `cls` với `STATIONS[n]["class"]` → sai một ký tự tên lớp là robot không bao giờ thấy trạm.
3. Hỏi Người 1 **chiều cao thật của khối số** (hoặc tự đo) → `marker_h_m` từng trạm (dùng tính khoảng cách, chỗ dừng).
4. Gửi `docs/YEU_CAU_NHOM_TRAIN.md`. Nếu nhóm cần ảnh từ camera robot, đưa Người 1 lệnh (họ tự chụp):
   `python training/capture_images.py --session robot_sang`.

**Đạt khi:** nhóm xác nhận `0=tram_1, 1=tram_2, 2=tram_3`; `pytest` 113 passed; `main.py --sim` chạy.

---

## Bước 2 · 08:30–10:30 · Code đọc ảnh, video, camera; xử lý khi mất hình

| Nguồn hình | Lệnh |
|---|---|
| Thư mục ảnh | `python main.py --fake-link --images anh_thu/ --fps 2 --no-floor` |
| Video | `python main.py --fake-link --video thu.mp4 --no-floor` |
| Webcam laptop / camera robot | `python main.py --fake-link` (không có picamera2 thì tự dùng webcam) |

`--fake-link` = Arduino giả (robot không chạy). `--no-floor` = tắt nhận diện sàn khi hình không quay từ camera robot.
Chưa có mô hình vẫn chạy được (màn hình báo "Chưa có mô hình YOLO").

**Thử mất hình** (webcam USB – **không rút cáp dẹt CSI khi Pi đang bật**):
1. `python main.py --fake-link` → nhấn **Enter**.
2. Rút webcam → ≤ 2 s màn hình đỏ **"LỖI HỆ THỐNG – ROBOT DỪNG / Lỗi camera"**, lệnh xuống Arduino là `S`.
3. Cắm lại → log `Mất hình -> mở lại camera`, màn hình **"SẴN SÀNG – Đã khắc phục lỗi – nhấn START"** (không tự chạy lại).

Kiểm thử: `python -m pytest tests/test_link_camera_dataset.py -k camera tests/test_mission.py -k fault -v`.

**Đạt khi:** 3 nguồn hình đều lên giao diện; mất hình → dừng + báo lỗi; có hình lại → chờ START.

---

## Bước 3 · 10:30–12:00 · Giao tiếp Pi–Arduino, timeout, lệnh dừng – thử bằng dữ liệu giả

| Pi → Arduino | Ý nghĩa | Arduino → Pi | Ý nghĩa |
|---|---|---|---|
| `W,fl,fr,rl,rr` | tốc độ 4 bánh (RPM), gửi đều 10–20 Hz | `I,ROBOT_FW,ver,hash,...` | thông tin + mã băm cấu hình |
| `S` | dừng | `T,ms,rpm×4,u×4,cờ,mode,vbat` | 20 Hz; cờ bit0 dừng khẩn, bit1 mất lệnh |
| `E,1` / `E,0` | dừng khẩn / xóa | `A,OK,…` / `A,ERR,…` | phản hồi |

**Arduino giả qua cổng serial ảo** (Linux / Pi / macOS; Windows dùng `--fake-link`):
```bash
python tools/fake_arduino.py --verbose                 # cửa sổ 1: in cổng, vd /dev/pts/5
python main.py --port /dev/pts/5 --images anh_thu/ --no-floor   # cửa sổ 2
```
| Thử | Cửa sổ 1 | Đúng khi |
|---|---|---|
| Kết nối | (mặc định) | không có cảnh báo "Firmware khác config"; RPM hiện bên phải màn hình |
| Mất cáp | `--drop-after 20` | sau ~21 s: "Mất kết nối Arduino (kiểm tra cáp USB)", robot dừng |
| Dừng khẩn | `--estop-at 15` | "DỪNG KHẨN CẤP"; nhấn **R** reset được |
| Timeout lệnh | tắt `main.py` | RPM về 0 sau 0.5 s |

Kiểm thử: `python -m pytest tests/test_link_camera_dataset.py -k serial -v`.
Có Arduino thật (kê robot lên): `python flash_arduino.py --install-cli` ("KHỚP cấu hình") →
`python tools/motor_test.py params` → rút USB khi bánh quay: bánh dừng ≤ 0.5 s.

**Đạt khi:** 4 thử nghiệm trên đúng.

---

## Bước 4 · 12:00–12:30 · Nghỉ & sao lưu code

```bash
git add -A && git commit -m "Ngày 1 – trưa: camera, giao tiếp Pi-Arduino"
git archive --format=zip -o ../robot_xuong_may_trua.zip HEAD    # chép sang USB / Drive
```

---

## Bước 5 · 12:30–13:00 · Kiểm tra tên/thứ tự lớp, định dạng file, dữ liệu đầu vào

Người 3 gộp và chia train/val/test. Việc của bạn là **kiểm tra** bộ đó trước khi họ bấm train:
```bash
python training/check_dataset.py nhan/dataset          # thư mục, hoặc:
python training/check_dataset.py nhan/dataset.zip      # file zip tải từ cloud
```
Kết quả mẫu (đạt):
```
data.yaml: data.yaml -> names = ['tram_1', 'tram_2', 'tram_3']
Các tập tìm thấy: train (images/train), val (images/val), test (images/test)
tập       ảnh  ảnh nền    tram_1    tram_2    tram_3
train     120       12        44        44        44
val        31        4        11        11        11
test       30        3        11        11        11
[CẢNH BÁO] 1 ảnh val/test GẦN TRÙNG ảnh train, ví dụ: [...]
KẾT LUẬN: ĐẠT – dùng được để train / đánh giá
```
| Báo | Nghĩa | Bạn làm gì |
|---|---|---|
| `[LỖI] tên/thứ tự lớp … KHÁC MODEL_CLASSES` | dataset cloud đặt sai tên / thứ tự | Người 3 sửa trước khi train |
| `[LỖI] … dòng nhãn sai định dạng` | mã lớp ≥ 3, tọa độ ngoài 0..1, thiếu số | gửi danh sách cho người gán nhãn |
| `[LỖI] tập … KHÔNG có đối tượng lớp …` | một tập thiếu hẳn 1 trạm | Người 3 chia lại |
| `[CẢNH BÁO] … GẦN TRÙNG ảnh train` / `buổi chụp nằm ở nhiều tập` | rò ảnh giữa học và kiểm tra | Người 3 chuyển cả buổi về một tập |
| `[CẢNH BÁO] … không phải tỉ lệ 4:3` | khác camera robot 640×480 | báo Người 1 (ảnh vẫn dùng được) |

Báo cáo lưu ở `logs/check_dataset_*.txt` → **gửi nguyên file cho Người 3**. Bạn không tự sửa nhãn, không chia lại.
Kiểu thư mục đọc được: Ultralytics (`images/train`, `labels/train`) và Roboflow (`train/images`, `valid/…`).

**Đạt khi:** `KẾT LUẬN: ĐẠT` (cảnh báo đã được Người 3 xem).

---

## Bước 6 · 13:00–14:30 · Logic 3 trạm, chờ 15 s, đếm ngược, xử lý mất dữ liệu

1. Thông số `robot/config.py`:
   | Thông số | Mặc định | Ý nghĩa |
   |---|---|---|
   | `ROUTE`, `ROUTE_LOOP` | `(1, 2, 3)`, `True` | thứ tự trạm, chạy vòng |
   | `wait_s` | 15.0 | thời gian chờ mỗi trạm |
   | `ARRIVE_DISTANCE_M` | 0.45 | khoảng cách coi là đã đến |
   | `STATION_CONF`, `CONFIRM_FRAMES` | 0.55, 3 | ngưỡng tin cậy, số khung xác nhận |
   | `TARGET_LOST_HOLD_S`, `TARGET_LOST_TIMEOUT_S` | 0.4, 1.5 | mất dấu khối ngắn / lâu |
   | `SEARCH_TIMEOUT_S`, `LOST_AUTO_RETRY_S` | 30, 10 | quá lâu không thấy trạm |
2. `python main.py --sim` → TRẠM 1 → 2 → 3 → 1, số lớn đếm **15 → 0**; phím **O** đặt vật cản, **N** bỏ qua chờ.
3. `python -m pytest tests/test_mission.py tests/test_mission_sim.py -v`.
4. Kiểm từng dòng xử lý mất dữ liệu:
   | Mất gì | Robot làm gì | Màn hình |
   |---|---|---|
   | Khối chập chờn < 0.4 s | giữ lệnh cũ, giảm 50 % | "Tạm mất dấu biển báo..." |
   | Mất khối 0.4–1.5 s | đứng chờ | "Mất dấu biển báo – đang chờ..." |
   | Mất khối > 1.5 s | quay tìm về phía thấy lần cuối | "Đang quay tìm biển báo trạm..." |
   | Không thấy trạm > 30 s | dừng, tự tìm lại sau 10 s | "KHÔNG TÌM THẤY TRẠM n" |
   | Mất hình > 2 s | dừng | "LỖI HỆ THỐNG – Lỗi camera" |
   | Mất Arduino > 1 s | dừng | "LỖI HỆ THỐNG – Mất kết nối Arduino" |
   | Pi treo > 0.5 s | Arduino tự dừng động cơ | (cờ timeout) |
5. Người 3 đang train: hỏi **ETA** mỗi 15 phút; ETA > 90 phút → sửa ô **Phút** bước 6 ở sheet kế hoạch (giờ sau tự tính lại).

**Đạt khi:** mô phỏng đúng thứ tự trạm, đếm ngược đúng 15 s; test mission qua; biết ETA.

---

## Bước 7 · 14:30–15:15 · Nhận .pt → kiểm tra lớp → xuất ONNX → tích hợp

1. Nhận từ Người 3 vào `nhan/`: `best.pt`, `args.yaml`, `results.csv`, tên phiên bản (vd `tram3_v1`).
2. Xuất (máy bạn đã cài `ultralytics onnx` ở phần chuẩn bị):
   ```bash
   python training/export_model.py --weights nhan/best.pt --version tram3_v1 \
          --args nhan/args.yaml --results nhan/results.csv
   ```
   Kết quả đúng (từ lần chạy thử thật):
   ```
   [export] Lớp OK: {0: 'tram_1', 1: 'tram_2', 2: 'tram_3'}
   [export] Đã tạo models/best.onnx và models/best.metadata.json
   [export] Phiên bản: tram3_v1  |  ảnh 416  |  3 lớp
   [check] best.onnx: phiên bản tram3_v1, lớp {...}, ảnh (416, 416), onnxruntime, 23 ms/khung
   [check] mã băm ONNX khớp metadata
   ```
   Thấy `[export] DỪNG: Tên/thứ tự lớp ... KHÁC config` → **trả Người 3 train lại**, không sửa code cho khớp.
3. `best.metadata.json` ghi: `model_version`, `names`, `imgsz`, mã băm `.pt`/`.onnx`, `train_args`, `metrics` (mAP…).
4. Chép sang Pi và kiểm tra:
   ```bash
   scp -r models pi@<ip-pi>:~/robot_xuong_may/
   ssh pi@<ip-pi> "cd robot_xuong_may && .venv/bin/python training/export_model.py --check models/best.onnx"
   ```
   Ghi **ms/khung trên Pi** (mục tiêu ≤ 250 ms). Chậm hơn → xuất lại `--imgsz 320`.
5. `python main.py --fake-link` → log `Nạp best.onnx (onnxruntime) … phiên bản tram3_v1`, màn hình **"Mô hình: tram3_v1"**.

**Đạt khi:** `best.onnx` + `best.metadata.json` đúng 3 nhãn; `--check` không có LỖI; Pi chạy được.

---

## Bước 8 · 15:15–16:45 · Chạy code bằng ảnh/video nhóm gửi; kiểm tra hiển thị, trạng thái, bộ đếm

1. **Chạy code robot trên tập test của Người 2** (cùng tập họ dùng):
   ```bash
   python training/test_model.py --eval nhan/dataset --split test
   ```
   ```
   lớp        đúng   nhầm  bỏ sót  báo giả  recall precision
   tram_1       11      0       0        0    1.00      1.00
   tram_2        9      0       2        1    0.82      0.90
   ```
   **Đối chiếu với kết quả Người 2** (họ đo bằng Ultralytics / cloud). Gần giống → tích hợp đúng.
   Khác nhiều → lỗi phía bạn (sai phiên bản file, ngưỡng `YOLO_CONF`, chép thiếu metadata).
   `logs/eval_test_*.csv` liệt kê từng ảnh **NHẦM TRẠM / BỎ SÓT / BÁO GIẢ** → gửi Người 2.
2. **Video / ảnh cảnh mới** Người 1, 2 gửi:
   ```bash
   python training/test_model.py --source video_moi.mp4
   python main.py --fake-link --video video_moi.mp4 --no-floor
   python main.py --fake-link --images anh_moi/ --fps 1 --no-floor
   ```
3. **Camera robot + khối thật** (Người 1 cầm khối, bạn xem màn hình):
   | Kiểm tra | Đúng khi |
   |---|---|
   | Khối trạm đích ở xa | băng xanh **"ĐANG DI CHUYỂN ĐẾN TRẠM n"**, "còn khoảng x m" |
   | Khoảng cách | sai ≤ 10 % so với thước (sai → `marker_h_m`, `CAMERA_HEIGHT_M`) |
   | Khối ≤ 0.45 m, giữa ảnh | **"ĐÃ ĐẾN TRẠM n"**, số lớn đếm **15 → 0** |
   | Hết 15 s | **"ĐANG DI CHUYỂN ĐẾN TRẠM n+1"** |
   | Khối của trạm khác | không phản ứng |
   | Che camera | "Lỗi camera" sau 2 s |
   | FPS | ≥ 4 trên Pi |

**Đạt khi:** code chạy được với ảnh/video bàn giao; bảng trên đúng; có CSV lỗi gửi nhóm.

---

## Bước 9 · 16:45–17:45 · Sửa lỗi code/tích hợp; nhận mô hình mới nếu nhóm train lại

| Hiện tượng | Thử | Loại lỗi | Ai sửa |
|---|---|---|---|
| Ultralytics của Người 2 cũng không thấy khối | – | **dữ liệu / mô hình** | nhóm train (chụp thêm, train lại) |
| Người 2 thấy, code robot không thấy | so phiên bản trong metadata, `YOLO_CONF` | **tích hợp** | **bạn** |
| Code thấy khối nhưng robot không phản ứng | so tên lớp log với `STATIONS` | **code / cấu hình** | **bạn** |
| Nhầm `tram_2` ↔ `tram_3` | cột "nhầm" | **dữ liệu** | nhóm train |
| Dừng quá xa / quá gần | đo thước | **cấu hình** `marker_h_m`, `ARRIVE_DISTANCE_M` | **bạn** |
| Laptop chạy, Pi lỗi | `--check` trên Pi | **môi trường** | **bạn** |
| FPS < 3 trên Pi | ms/khung | **hiệu năng** → xuất `--imgsz 320` | **bạn** |

Nhóm train lại → nhận `.pt` mới + phiên bản mới:
```bash
python training/export_model.py --weights nhan_v2/best.pt --version tram3_v2 --args nhan_v2/args.yaml --results nhan_v2/results.csv
python training/test_model.py --eval nhan/dataset --split test        # CÙNG tập test, so với v1
```

**Đạt khi:** mọi lỗi đã xếp loại; lỗi phía code đã sửa hoặc ghi lại; chốt phiên bản dùng tiếp.

---

## Bước 10 · 17:45–18:00 · Đóng gói code, hướng dẫn chạy, ghi phiên bản mô hình

```bash
python -m pytest -q
git add -A && git commit -m "Ngày 1: tích hợp mô hình tram3_vX"
git archive --format=zip -o ../robot_xuong_may_ngay1.zip HEAD
```
Ghi biên bản: `model_version` và `sha256_onnx` (trong `models/best.metadata.json`), bảng đánh giá test, lỗi tồn đọng.
**Chưa đánh giá tự hành xe thật** (đúng kế hoạch) → ngày 2: `flash_arduino.py`, `motor_test.py`, học sàn (**F**),
dạy lộ trình (**M → T**), chạy tự động có giám sát (README §5).

---

## Phụ lục A – Lệnh của bạn

| Việc | Lệnh |
|---|---|
| Kiểm thử | `python -m pytest -q` |
| Mô phỏng | `python main.py --sim` |
| Chạy bằng ảnh / video | `python main.py --fake-link --images DIR --no-floor` · `--video FILE` |
| Arduino giả | `python tools/fake_arduino.py [--drop-after 20] [--estop-at 15]` |
| Kiểm tra dữ liệu nhóm gửi | `python training/check_dataset.py nhan/dataset(.zip)` |
| Xuất ONNX + metadata | `python training/export_model.py --weights best.pt --version tram3_v1` |
| Kiểm tra ONNX | `python training/export_model.py --check models/best.onnx` |
| Đánh giá trên test | `python training/test_model.py --eval nhan/dataset --split test` |
| Nạp Arduino | `python flash_arduino.py` |

Công cụ trong repo **dành cho nhóm train** (bạn không cần chạy): `training/capture_images.py`,
`training/split_dataset.py`, `training/train_yolo.py` (dự phòng nếu không train trên cloud).

## Phụ lục B – Lỗi hay gặp

| Thông báo | Nguyên nhân | Cách xử lý |
|---|---|---|
| `Chưa có mô hình YOLO – chỉ chạy được chế độ tay` | thiếu `models/best.onnx` / `.metadata.json` | bước 7 |
| `Thiếu tên lớp: cần file best.metadata.json` | chép thiếu file | chép cả thư mục `models/` |
| `Mô hình KHÔNG có lớp 'tram_1'` / `Thứ tự lớp sai` | nhóm đặt tên khác | nhóm train lại đúng tên |
| `No module named onnxruntime` | thiếu thư viện | `pip install onnxruntime` |
| `No module named ultralytics` khi xuất | chưa cài gói xuất | `pip install ultralytics onnx onnxslim` |
| `No module named picamera2` trong venv | venv thiếu `--system-site-packages` | tạo lại theo `install_pi.sh` |
| `Permission denied /dev/ttyACM0` | thiếu quyền | `sudo usermod -aG dialout $USER`, đăng nhập lại |
| `Firmware Arduino khác config.py` | sửa config chưa nạp lại | `python flash_arduino.py` |
| `fake_arduino.py` lỗi trên Windows | Windows không có pty | dùng `--fake-link` |

## Phụ lục C – Rủi ro phía code và dự phòng

| Rủi ro | Dấu hiệu | Dự phòng |
|---|---|---|
| Mô hình bàn giao trễ (ETA dài) | 14:30 chưa có `.pt` | làm trước bước 8 bằng `main.py --sim` và ảnh thử; sửa ô Phút trong Excel |
| Pi chạy chậm ở 416 | > 250 ms/khung | xuất lại `--imgsz 320`; tản nhiệt + quạt |
| Nhầm phiên bản mô hình | dòng "Mô hình:" khác phiên bản nhóm báo | luôn xuất với `--version`, so `sha256_onnx` |
| Ảnh camera ngoài khác camera robot | Người 2 đo tốt, camera robot kém | báo nhóm chụp thêm bằng camera robot, train lại |
