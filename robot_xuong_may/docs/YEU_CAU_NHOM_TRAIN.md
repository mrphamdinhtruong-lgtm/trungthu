# YÊU CẦU GỬI NHÓM TRAIN (3 người) – TỪ NGƯỜI VIẾT CODE

Để code robot dùng được mô hình ngay, nhóm train bàn giao **đúng các điều dưới đây**.
Mọi việc chụp, gán nhãn, gộp, chia và train là của nhóm; người viết code chỉ **kiểm tra và tích hợp**.

## 1. Tên lớp – KHÔNG ĐƯỢC ĐỔI

| Mã | Tên lớp (đúng chính tả, chữ thường, có gạch dưới) |
|---|---|
| 0 | `tram_1` |
| 1 | `tram_2` |
| 2 | `tram_3` |

Một mô hình chung cho cả 3 trạm. **Không** dùng `tram1`, `Tram 1`, `TRAM_1`… (robot sẽ từ chối mô hình).

## 2. Ảnh

* `.jpg`, tỉ lệ **4:3** (giống camera robot 640×480), cạnh nhỏ ≥ 480 px.
* **Mỗi buổi chụp một thư mục** (vd `A_sang/`, `B_chieu/`): cùng chỗ, cùng giờ, cùng người.
* Nhiều khoảng cách (0.3–6 m), lệch trái/phải, nghiêng, bị che một phần, sáng/tối; ~10 % ảnh không có khối.
* Nên có ảnh chụp **bằng camera robot ở đúng độ cao**: `python training/capture_images.py --session robot_sang`.

## 3. Gán nhãn

* Định dạng **YOLO**: mỗi ảnh một `.txt` cùng tên, mỗi dòng `mã_lớp cx cy w h` (số 0..1).
* Khoanh **sát** khối số; khối bị cắt ở mép ảnh vẫn khoanh. **Không** khoanh thùng rác / bàn của trạm.
* Mỗi phần A/B/C đều có đủ 3 lớp.

## 4. Gộp & chia dữ liệu (Người 3)

* Chia **train / val / test theo buổi** – một buổi chỉ nằm trong một tập; ảnh gần trùng không được ở cả train và test.
* Tập **test giữ riêng**, không dùng để chọn mô hình.
* `data.yaml`: `names: {0: tram_1, 1: tram_2, 2: tram_3}`.
* (Nếu không chia trên cloud: `python training/split_dataset.py --src dataset/labeled --val 0.2 --test 0.1`.)
* Người viết code sẽ chạy `training/check_dataset.py` – có `[LỖI]` thì nhóm sửa trước khi train.

## 5. Train (Người 3)

`YOLO11n Detect` · **50 epochs** · ảnh **416** · batch **Auto** · **tắt lật ngang** (fliplr = 0, số bị lật sai hình).
Báo ETA cho người viết code mỗi 15 phút.

## 6. Bàn giao cho người viết code

| Thứ | Ai | Hạn (theo kế hoạch) |
|---|---|---|
| Đợt ảnh đầu (để thử code đọc ảnh) | Người 1 | 10:30 |
| Bộ dữ liệu gộp + `data.yaml` (thư mục hoặc zip tải từ cloud) | Người 3 | 13:00 |
| `best.pt` + `args.yaml` + `results.csv` (+ `confusion_matrix.png`) | Người 3 | khi train xong (~14:30) |
| **Tên phiên bản** mô hình: `tram3_v1`, `tram3_v2`… (tăng mỗi lần train lại) | Người 3 | cùng `best.pt` |
| Kết quả kiểm tra trên tập test (số đúng / nhầm / bỏ sót mỗi trạm) | Người 2 | 15:15 |
| Ảnh / video cảnh MỚI (không có trong train), video 4:3 nếu được | Người 1, 2 | 15:15 |
| Bản sao ảnh gốc, nhãn, `.pt` + cấu hình cuối ngày | cả nhóm | 18:00 |

Người viết code trả lại: `best.onnx` + `best.metadata.json` (đúng 3 nhãn, có phiên bản), bảng đánh giá bằng code robot,
danh sách lỗi (`logs/eval_test_*.csv`) để nhóm chụp bổ sung / sửa nhãn.
