# HUẤN LUYỆN YOLO NHẬN DIỆN 3 TRẠM (theo kế hoạch train ngày 1)

> Hướng dẫn từng giờ cho người viết code: `docs/HUONG_DAN_NGAY_1.md`.

## 1. Quyết định đã chốt

| Hạng mục | Chốt |
|---|---|
| Lớp (đúng THỨ TỰ) | `0 = tram_1`, `1 = tram_2`, `2 = tram_3` – trùng `MODEL_CLASSES` trong `robot/config.py` |
| Mô hình | **MỘT** mô hình YOLO11n Detect chung cho cả 3 trạm (không tách 3 mô hình) |
| Huấn luyện | 50 epochs, ảnh 416, batch Auto |
| Bàn giao | `best.pt` + `args.yaml` + `results.csv` + tên phiên bản (vd `tram3_v1`) |
| Robot dùng | `models/best.onnx` + `models/best.metadata.json` (xuất bằng `training/export_model.py`) |

Mô hình 3 lớp **không có** lớp người / vật cản → robot phát hiện vật cản bằng **nhận diện sàn** (`docs/NAVIGATION.md`).
Muốn YOLO nhận cả người / vật cản: thêm lớp `3: vat_can`, `4: nguoi` vào `training/data.yaml` và `MODEL_CLASSES`, train lại.

## 2. Khối số / biển trạm

* Mỗi trạm khác nhau cả **số, màu và hình** → YOLO phân biệt tốt, ít nhầm trạm.
* Đo **chiều cao thật** của khối (m) → `STATIONS[n]["marker_h_m"]` (dùng ước lượng khoảng cách, quyết định chỗ dừng).
* Đặt khối **ngang tầm camera robot**, mặt số quay về hướng robot đi tới.

## 3. Chụp ảnh

* Ưu tiên chụp **bằng chính camera robot ở đúng độ cao** (`python training/capture_images.py --session <buoi>`):
  ảnh camera điện thoại / camera ngoài khác màu sắc, góc rộng, độ nét → mô hình có thể kém khi chạy trên robot.
  Nếu dùng camera ngoài: chụp ở độ cao 20–30 cm như camera robot, khung hình 4:3, và **thêm ít nhất 30 % ảnh từ camera robot**.
* Mỗi **buổi** chụp (một chỗ / một khung giờ / một người) để **một thư mục riêng** – dùng để chia train/val/test theo buổi.
* Mỗi trạm 300–500 ảnh: khoảng cách 0.3–6 m, lệch trái/phải, nghiêng, bị che một phần, ánh sáng khác nhau;
  ~10 % ảnh nền không có khối nào.

```bash
python training/capture_images.py --session A_sang --prefix tram_1   # -> dataset/raw/A_sang/
python training/capture_images.py --session B_chieu --interval 1      # tự chụp mỗi 1 s
```

## 4. Gán nhãn (định dạng YOLO)

* Khung **ôm sát** khối số; khối bị cắt ở mép ảnh vẫn gán; đúng mã lớp 0/1/2.
* **Không** khoanh thùng rác / bàn của trạm.
* Công cụ: Roboflow / Ultralytics Platform / CVAT / Label Studio – **xuất định dạng YOLO** (mỗi ảnh một `.txt`: `lớp cx cy w h`).

## 5. Gộp & chia dữ liệu THEO BUỔI

```bash
# dataset/labeled/<buổi>/*.jpg + *.txt  (phần A/B/C của 3 người, mỗi buổi 1 thư mục con)
python training/split_dataset.py --src dataset/labeled --val 0.2 --test 0.1
```
Script báo: số đối tượng mỗi lớp trong từng tập, tập thiếu lớp, dòng nhãn sai mã lớp / sai tọa độ,
**ảnh val/test gần trùng ảnh train**. Mỗi buổi chỉ nằm trong 1 tập.

## 6. Huấn luyện

Trên cloud (nhóm train) hoặc máy có GPU / Colab:
```bash
pip install -r requirements-train.txt
python training/train_yolo.py                 # YOLO11n, 50 epochs, 416, batch Auto (theo kế hoạch)
```
Tăng cường dữ liệu đã chọn sẵn: **tắt lật ngang** (số bị lật sai hình), xoay nhẹ, đổi sáng/màu.
Kết quả: `runs/detect/tram3/weights/best.pt`, `args.yaml`, `results.csv`, `confusion_matrix.png`.

## 7. Nhận `.pt` → xuất ONNX cho robot

```bash
python training/export_model.py --weights best.pt --version tram3_v1 --args args.yaml --results results.csv
# -> models/best.onnx, models/best.metadata.json, models/best.pt
python training/export_model.py --check models/best.onnx     # kiểm tra lại (lớp, mã băm, chạy thử)
```
Sai tên / thứ tự lớp → script **dừng, không xuất**. Chép thư mục `models/` sang Pi; trên Pi chỉ cần `onnxruntime`.

## 8. Đánh giá bằng đúng code robot

```bash
python training/test_model.py --eval dataset --split test       # đúng / nhầm trạm / bỏ sót / báo giả theo từng trạm
python training/test_model.py --source video_moi.mp4            # xem trực tiếp, FPS, khoảng cách ước lượng
```
Danh sách lỗi ghi vào `logs/eval_test_*.csv` → gửi nhóm train để chụp bổ sung / sửa nhãn.

## 9. Hiệu chỉnh

| Thông số (`robot/config.py`) | Cách chỉnh |
|---|---|
| `marker_h_m` | chiều cao thật của khối số |
| `ARRIVE_DISTANCE_M` | đặt robot đúng chỗ muốn dừng, đọc "Khoảng cách" trên màn hình |
| `STATION_CONF`, `CONFIRM_FRAMES` | báo giả nhiều → tăng; khó nhận ra → giảm |
| `YOLO_IMGSZ` | ONNX tự dùng kích thước lúc xuất; Pi chậm → xuất lại `--imgsz 320` |
