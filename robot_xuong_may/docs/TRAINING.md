# HUẤN LUYỆN YOLO NHẬN DIỆN TRẠM & VẬT CẢN

## 1. Lớp đối tượng (`training/data.yaml`)

| ID | Lớp | Ý nghĩa | Dùng trong robot |
|---|---|---|---|
| 0 | `tram1` | Biển báo TRẠM 1 | đích đến, ước lượng khoảng cách |
| 1 | `tram2` | Biển báo TRẠM 2 | |
| 2 | `tram3` | Biển báo TRẠM 3 | |
| 3 | `vat_can` | Thùng, xe đẩy, cuộn vải, ghế, hộp... trên đường đi | dừng khi gần |
| 4 | `nguoi` | Người đứng / đi trên đường | dừng khi gần |

Thêm trạm: thêm lớp trong `data.yaml`, khai báo trong `STATIONS` và `ROUTE` của `robot/config.py`.

## 2. Thiết kế biển báo trạm (rất quan trọng cho độ chính xác)

* Kích thước gợi ý **20 × 20 cm** (đúng với `marker_h_m = 0.20`), in giấy mờ (không bóng) dán lên tấm fomex.
* Mỗi trạm **khác nhau cả số, màu và hình**: TRẠM 1 – nền xanh dương số "1" tròn; TRẠM 2 – nền xanh lá số "2" tam giác;
  TRẠM 3 – nền đỏ số "3" vuông. YOLO phân biệt rất tốt khi khác cả màu + hình.
* Treo **ngang tầm camera**, mặt biển quay về hướng robot đi tới; mỗi trạm phải **nhìn thấy được từ trạm trước**
  (nếu bị khuất, thêm 1 biển "trạm trung gian" với `wait_s = 0` vào `ROUTE`).

## 3. Thu thập ảnh (trên Pi, bằng chính camera của robot)

```bash
python training/capture_images.py --prefix tram1            # SPACE chụp, A tự chụp
python training/capture_images.py --interval 1 --prefix nen # tự chụp mỗi 1 s khi đẩy robot đi quanh xưởng
```

| Yêu cầu | Số lượng gợi ý |
|---|---|
| Mỗi lớp trạm | 300–500 ảnh: khoảng cách 0.3–6 m, lệch trái/phải, nghiêng, bị che một phần |
| `vat_can`, `nguoi` | 300–600 ảnh: nhiều loại vật / người, nhiều khoảng cách, đặc biệt **ở gần** |
| Ảnh nền (không có gì) | ~10 % tổng số ảnh – giảm báo giả |
| Ánh sáng | sáng / tối / ngược sáng / đèn huỳnh quang nhấp nháy |

## 4. Gán nhãn

Chọn một công cụ, **xuất định dạng YOLO** (mỗi ảnh một file `.txt`: `lớp cx cy w h` chuẩn hóa):
* **Roboflow** (web, dễ nhất – xuất "YOLOv8" là dùng được ngay với `data.yaml`),
* **CVAT** / **Label Studio** / **LabelImg**.

Quy tắc: khung **ôm sát** biển báo / vật; vật bị cắt ở mép ảnh vẫn gán nhãn; đúng thứ tự lớp như bảng trên.
**Không** gán nhãn `vat_can` cho thùng rác / bàn **của chính trạm** (nếu không robot sẽ dừng trước trạm vì "gặp vật cản").

Chia train/val (nếu công cụ chưa chia):
```bash
python training/split_dataset.py --src dataset/labeled --val 0.2
```

## 5. Huấn luyện (laptop có GPU hoặc Google Colab – không train trên Pi 4)

```bash
pip install -r requirements-train.txt
python training/train_yolo.py --epochs 120 --imgsz 320          # YOLO11n
python training/train_yolo.py --model yolov8n.pt --imgsz 320     # hoặc YOLOv8n
```

**Google Colab** (Runtime → GPU):
```python
!git clone <địa chỉ repo của bạn>
%cd trungthu/robot_xuong_may
!pip install -q ultralytics
# tải dataset/ lên (zip) rồi giải nén vào robot_xuong_may/dataset
!python training/train_yolo.py --epochs 120 --imgsz 320 --export
```

Cấu hình tăng cường dữ liệu đã chọn sẵn cho biển báo có chữ số: **tắt lật ngang** (`fliplr=0`), xoay nhẹ, đổi độ sáng/màu.
Kết quả cho báo cáo: `runs/detect/tram_xuong_may/` (results.png, confusion_matrix.png, PR_curve.png).
Mục tiêu: mAP50 ≥ 0.90 cho các lớp trạm.

## 6. Xuất mô hình cho Raspberry Pi 4

```bash
python training/export_model.py --weights runs/detect/tram_xuong_may/weights/best.pt --imgsz 320
# -> models/best_ncnn_model/   (chép thư mục models/ sang Pi nếu xuất trên laptop)
```
`robot/config.py`: `MODEL_PATH = "models/best_ncnn_model"`, `YOLO_IMGSZ = 320`.

NCNN là định dạng Ultralytics khuyến nghị cho CPU ARM của Pi. Tốc độ thực tế phụ thuộc mô hình và nhiệt độ CPU –
**đo trên robot**:
```bash
python training/test_model.py                 # camera trực tiếp, hiện FPS + khoảng cách ước lượng
python training/test_model.py --source dataset/images/val
```
Nếu FPS thấp: giữ `imgsz 320`, dùng bản nano (`n`), gắn tản nhiệt + quạt cho Pi (CPU bị hạ xung khi > 80 °C).

## 7. Hiệu chỉnh sau khi có mô hình

| Thông số (`robot/config.py`) | Cách chỉnh |
|---|---|
| `marker_h_m` | chiều cao thật của biển báo |
| `ARRIVE_DISTANCE_M` | đặt robot đúng chỗ muốn dừng, đọc "Khoảng cách" trên màn hình |
| `STATION_CONF`, `CONFIRM_FRAMES` | báo giả nhiều → tăng; khó nhận ra → giảm |
| `OBST_STOP_DISTANCE_M`, `OBST_CORRIDOR` | khoảng cách & bề rộng vùng nguy hiểm phía trước |
| `CAMERA_HEIGHT_M`, `CAMERA_TILT_DEG` | đo thực tế – quyết định độ chính xác khoảng cách vật cản |

**Giới hạn cần ghi trong báo cáo:** camera + YOLO chỉ phát hiện được loại vật **đã có trong dữ liệu huấn luyện**;
vật lạ, vật trong suốt, vật quá thấp sát sàn có thể không được nhận ra. Robot chỉ "nhìn" phía trước,
không thấy phía sau / hai bên (vì vậy mặc định không lùi: `DEPART_REVERSE_S = 0`).
