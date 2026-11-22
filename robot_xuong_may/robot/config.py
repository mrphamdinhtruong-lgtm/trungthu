# -*- coding: utf-8 -*-
"""
=====================================================================
 CẤU HÌNH TRUNG TÂM – ROBOT THU GOM RÁC / VẢI VỤN XƯỞNG MAY
=====================================================================
Đây là NGUỒN CẤU HÌNH DUY NHẤT của toàn bộ hệ thống:
  * Chương trình Python trên Raspberry Pi 4 (xử lý ảnh, nhiệm vụ, hiển thị)
  * Firmware Arduino UNO (bộ điều khiển tốc độ SAC-DOB cho 4 động cơ)
    -> flash_arduino.py tự sinh file firmware/robot_firmware/config_gen.h
       từ các thông số trong file này rồi biên dịch + nạp xuống Arduino.

Quy ước:
  * Thứ tự bánh xe luôn là: 0 = FL (trước-trái), 1 = FR (trước-phải),
                              2 = RL (sau-trái),   3 = RR (sau-phải)
  * Hệ trục robot: vx > 0 tiến, vy > 0 sang TRÁI, wz > 0 quay NGƯỢC chiều kim đồng hồ
  * Sau khi sửa nhóm [B] hoặc [C] phải chạy lại:  python flash_arduino.py
"""

# =====================================================================
# [A] CƠ KHÍ – BÁNH MECANUM & KHUNG XE
# =====================================================================
WHEEL_DIAMETER_M = 0.097            # Bánh mecanum 97 mm
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
HALF_WHEELBASE_M = 0.100            # Lx: 1/2 khoảng cách trục bánh trước - trục bánh sau (ĐO LẠI!)
HALF_TRACK_M = 0.115                # Ly: 1/2 khoảng cách tâm bánh trái - tâm bánh phải (ĐO LẠI!)

# Động cơ JGB37-520 12V 110 RPM có encoder Hall 2 kênh (ở đây chỉ dùng kênh A)
MOTOR_RPM_NOMINAL = 110.0           # Tốc độ không tải danh định trục ra (RPM) – dùng để chuẩn hóa
ENCODER_PPR_MOTOR = 11              # Số xung/vòng trên trục động cơ (trước hộp số)
GEAR_RATIO = 90.0                   # Tỉ số truyền hộp số (110 RPM ~ 1:90) – KIỂM TRA bằng tools/motor_test.py count
ENCODER_EDGES = 2                   # Đếm cả cạnh lên & xuống của kênh A (CHANGE) -> x2
WHEEL_RPM_LIMIT = 85.0              # Giới hạn tốc độ đặt mỗi bánh (L298N sụt áp ~2 V nên không đạt 110 RPM)

# =====================================================================
# [B] ARDUINO UNO – CHÂN, TRUYỀN THÔNG, AN TOÀN  (sửa xong phải nạp lại)
# =====================================================================
SERIAL_PORT = "auto"                # "auto" hoặc "/dev/ttyACM0", "/dev/ttyUSB0", "COM5"...
SERIAL_BAUD = 115200
CONTROL_PERIOD_MS = 20              # Chu kỳ vòng điều khiển tốc độ (20 ms = 50 Hz)
TELEMETRY_PERIOD_MS = 50            # Chu kỳ gửi dữ liệu về Pi (50 ms = 20 Hz)
CMD_TIMEOUT_MS = 500                # Mất lệnh từ Pi quá thời gian này -> Arduino tự dừng động cơ
HW_WATCHDOG = True                  # Bật watchdog phần cứng AVR (treo chương trình -> tự reset)

# Đảo chiều quay từng động cơ nếu đấu dây ngược (FL, FR, RL, RR)
# Bên phải thường lắp đối xứng nên đảo chiều. Kiểm tra bằng: python tools/motor_test.py dir
MOTOR_INVERT = (False, True, False, True)

PWM_DEADZONE = 28                   # Bù vùng chết PWM (0..255) – động cơ bắt đầu quay từ giá trị này
PWM_TIMER1_FAST = True              # Đưa PWM chân D9/D10 lên ~980 Hz cho giống D5/D6
ACCEL_LIMIT_RPM_S = 300.0           # Giới hạn gia tốc của tốc độ đặt (RPM/s) – chống trượt bánh
SPEED_LPF_ALPHA = 0.5               # Hệ số lọc thông thấp tốc độ đo (0..1, 1 = không lọc)
IDLE_SPEED_RPM = 2.0                # Dưới ngưỡng này khi đặt 0 -> coi như đứng yên, ngắt PWM
DIR_SWITCH_RPM = 4.0                # Encoder 1 kênh: chỉ đổi dấu đếm khi bánh gần dừng

ESTOP_INPUT_ENABLE = False          # True: dùng nút dừng khẩn (tiếp điểm NC) nối A4 - GND
BATTERY_SENSE_ENABLE = True         # True: đo áp pin 3S qua cầu phân áp vào A5
BATTERY_DIVIDER_R1_K = 10.0         # Điện trở trên (pin+ -> A5), kΩ
BATTERY_DIVIDER_R2_K = 4.7          # Điện trở dưới (A5 -> GND), kΩ
BATTERY_WARN_V = 10.8               # 3S: 3.6 V/cell -> cảnh báo pin yếu
BATTERY_CRIT_V = 10.2               # 3S: 3.4 V/cell -> dừng robot, yêu cầu sạc

# =====================================================================
# [C] BỘ ĐIỀU KHIỂN TỐC ĐỘ ĐỘNG CƠ: SAC + DOB (THÍCH NGHI)  (sửa xong nạp lại,
#     hoặc chỉnh trực tiếp khi đang chạy bằng: python tools/motor_test.py tune <ten> <gia_tri>)
# Tín hiệu đã chuẩn hóa: y = RPM / MOTOR_RPM_NOMINAL, u = duty -1..1
# =====================================================================
CONTROLLER_MODE = 2                 # 0 = PI (để so sánh), 1 = SAC, 2 = SAC + DOB (mặc định)

# --- SAC (Simple Adaptive Control – Kaufman/Barkana/Sobel) ---
SAC_TAU_M = 0.15                    # Hằng số thời gian mô hình mẫu (s): đáp ứng mong muốn
SAC_GAMMA_I = (5.0, 10.0, 10.0)       # Tốc độ thích nghi phần tích phân  [Ke, Kx, Ku]
SAC_GAMMA_P = (1.0, 0.3, 0.3)       # Hệ số thích nghi phần tỉ lệ       [Ke, Kx, Ku]
SAC_SIGMA = 0.1                     # Hệ số rò sigma-modification (chống trôi tham số)
SAC_K0 = (1.0, 0.4, 0.8)            # Độ lợi ban đầu                     [Ke, Kx, Ku]
SAC_K_MIN = (0.0, -1.5, -1.5)       # Chặn dưới độ lợi
SAC_K_MAX = (8.0, 3.0, 3.0)         # Chặn trên độ lợi
SAC_E_DEADBAND = 0.01               # Vùng chết sai số: |e| nhỏ hơn thì ngừng thích nghi (chống nhiễu)

# --- DOB (Disturbance Observer – bộ quan sát nhiễu) ---
DOB_KN = 0.90                       # Hệ số khuếch đại mô hình danh định  Pn(s) = Kn / (tau_n s + 1)
DOB_TAU_N = 0.10                    # Hằng số thời gian mô hình danh định (s) – nhận dạng bằng motor_test.py ident
DOB_TAU_Q = 0.12                    # Hằng số thời gian bộ lọc Q(s) = 1/(tau_q s + 1)
DOB_LIMIT = 0.6                     # Giới hạn biên độ nhiễu ước lượng được bù

# --- PI (chỉ dùng để so sánh trong báo cáo) ---
PI_KP = 1.0
PI_KI = 8.0

# =====================================================================
# [D] CAMERA & YOLO (Raspberry Pi)
# =====================================================================
CAMERA_BACKEND = "auto"             # "auto" | "picamera2" (Camera Module 2) | "opencv" (USB webcam)
CAMERA_INDEX = 0                    # Chỉ số webcam khi dùng OpenCV
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FLIP = ""                    # "" | "h" | "v" | "hv" (lật ảnh nếu lắp camera ngược)
CAMERA_HFOV_DEG = 62.2              # Góc nhìn ngang Camera Module 2
CAMERA_VFOV_DEG = 48.8              # Góc nhìn dọc Camera Module 2
CAMERA_HEIGHT_M = 0.25              # Chiều cao tâm ống kính so với sàn (ĐO LẠI!)
CAMERA_TILT_DEG = 0.0               # Góc chúc xuống của camera (+ = chúc xuống)

MODEL_PATH = "models/best.onnx"     # best.onnx + best.metadata.json (xuất bằng training/export_model.py)
                                    # cũng nhận best.pt / *_ncnn_model (cần cài ultralytics)
YOLO_IMGSZ = 416                    # Theo kế hoạch train: ảnh 416 (ONNX tự lấy kích thước trong metadata)
YOLO_CONF = 0.45                    # Ngưỡng tin cậy chung
YOLO_IOU = 0.50

# Tên & THỨ TỰ lớp phải TRÙNG với training/data.yaml và mô hình nhóm train bàn giao
# (kế hoạch train: 0 = tram_1, 1 = tram_2, 2 = tram_3). Khi xuất / nạp mô hình sẽ kiểm tra.
MODEL_CLASSES = ("tram_1", "tram_2", "tram_3")
# Lớp vật cản (nếu sau này train thêm). Mô hình 3 lớp hiện tại không có -> vật cản do NHẬN DIỆN SÀN phát hiện.
OBSTACLE_CLASSES = ("vat_can", "nguoi")

# =====================================================================
# [E] TRẠM & LỘ TRÌNH
# =====================================================================
# class       : tên lớp YOLO của biển báo trạm
# name        : chữ hiển thị trên màn hình
# wait_s      : thời gian dừng chờ công nhân bỏ rác / vải vụn
# marker_h_m  : chiều cao thật của biển báo / KHỐI SỐ (m) – dùng ước lượng khoảng cách (ĐO LẠI!)
# search_dir  : chiều quay khi tìm trạm: +1 = quay trái (CCW), -1 = quay phải (CW)
# heading_deg : (tùy chọn) hướng la bàn gần đúng nhìn về trạm, None = không dùng
STATIONS = {
#   (kế hoạch ngày 1: chờ 15 s; yêu cầu đề tài ban đầu 20 s -> chỉ sửa wait_s)
    1: {"class": "tram_1", "name": "TRẠM 1", "wait_s": 15.0, "marker_h_m": 0.20, "search_dir": -1, "heading_deg": None},
    2: {"class": "tram_2", "name": "TRẠM 2", "wait_s": 15.0, "marker_h_m": 0.20, "search_dir": -1, "heading_deg": None},
    3: {"class": "tram_3", "name": "TRẠM 3", "wait_s": 15.0, "marker_h_m": 0.20, "search_dir": -1, "heading_deg": None},
}
ROUTE = (1, 2, 3)                   # Thứ tự đi qua các trạm
ROUTE_LOOP = True                   # True: hết trạm 3 quay lại trạm 1 (chạy vòng liên tục)

# =====================================================================
# [F] DẪN HƯỚNG BẰNG THỊ GIÁC (visual servoing)
# =====================================================================
STATION_CONF = 0.55                 # Ngưỡng tin cậy để công nhận biển báo trạm
CONFIRM_FRAMES = 3                  # Số khung hình liên tiếp thấy trạm mới chuyển sang tiếp cận
W_SEARCH = 0.55                     # Tốc độ quay khi tìm trạm (rad/s)
W_SEARCH_SLOW = 0.20                # Quay chậm khi vừa thoáng thấy trạm (chờ xác nhận)
SEARCH_TIMEOUT_S = 30.0             # Quá thời gian không tìm thấy -> báo lỗi
LOST_AUTO_RETRY_S = 10.0            # Tự tìm lại sau bao lâu khi báo không thấy trạm (0 = chờ người vận hành)

APPROACH_MODE = "rotate"            # "rotate": quay để căn giữa | "strafe": đi ngang (mecanum) để căn giữa
V_APPROACH_MAX = 0.22               # Tốc độ tiến tối đa (m/s)
V_APPROACH_MIN = 0.06               # Tốc độ tiến khi đã gần trạm (m/s)
K_W_CENTER = 1.0                    # Độ lợi quay căn giữa (rad/s trên đơn vị lệch -1..1)
W_MAX = 0.8                         # Giới hạn tốc độ quay (rad/s)
K_VY_CENTER = 0.15                  # Độ lợi đi ngang căn giữa (m/s) – chế độ strafe
EX_TURN_ONLY = 0.6                  # Lệch tâm lớn hơn ngưỡng này -> chỉ quay, không tiến
ARRIVE_DISTANCE_M = 0.45            # Khoảng cách coi là ĐÃ ĐẾN trạm (m)
SLOW_DISTANCE_M = 1.20              # Bắt đầu giảm tốc từ khoảng cách này (m)
ARRIVE_CENTER_TOL = 0.25            # Lệch tâm cho phép khi dừng tại trạm
ARRIVE_FRAMES = 3                   # Số khung hình liên tiếp thỏa điều kiện mới xác nhận đến trạm
TRACK_SMOOTH = 0.5                  # Lọc làm mượt vị trí biển báo (0..1)
TARGET_LOST_HOLD_S = 0.4            # Mất dấu ngắn: giữ lệnh cũ (giảm tốc) trong thời gian này
TARGET_LOST_TIMEOUT_S = 1.5         # Mất dấu lâu hơn -> quay lại chế độ tìm trạm

DEPART_REVERSE_S = 0.0              # Lùi khỏi trạm trước khi tìm trạm kế (0 = không lùi, camera không thấy phía sau)
DEPART_REVERSE_V = 0.10             # Tốc độ lùi (m/s)

# --- Điều khiển tay (bàn phím trên giao diện) ---
MANUAL_V = 0.20                     # m/s tiến/lùi/đi ngang
MANUAL_W = 0.80                     # rad/s quay

# --- La bàn (giữ hướng) ---
HEADING_KP = 1.5                    # rad/s trên rad sai lệch hướng
HEADING_TOL_DEG = 8.0               # Sai số hướng chấp nhận khi quay về hướng gợi ý

# =====================================================================
# [G] PHÁT HIỆN VẬT CẢN BẰNG CAMERA
# =====================================================================
OBST_CONF = 0.50                    # Ngưỡng tin cậy vật cản
OBST_STOP_DISTANCE_M = 0.80         # Vật cản trong LÀN ĐƯỜNG robot (bề rộng thật) gần hơn khoảng này -> DỪNG
                                    # (khoảng cách ước lượng từ mép dưới khung = điểm chạm sàn)
OBST_ON_FRAMES = 2                  # Số khung liên tiếp thấy vật cản mới dừng (chống báo giả)
OBST_CLEAR_S = 1.5                  # Vật cản biến mất liên tục bao lâu mới chạy tiếp
OBST_CHECK_IN_SEARCH = True         # Kiểm tra vật cản cả khi đang quay tìm trạm
OBST_CHECK_IN_MANUAL = True         # Chế độ tay: chặn lệnh tiến khi có vật cản

# =====================================================================
# [G2] DẪN ĐƯỜNG BẰNG CAMERA – NHẬN DIỆN MẶT SÀN TRỐNG (không dán line)
# =====================================================================
FLOOR_ENABLE = True                 # Bật nhận diện sàn trống (phát hiện cả vật cản lạ YOLO chưa học)
FLOOR_TYPE = "auto"                 # "tile" = gạch / đá hoa bóng | "concrete" = bê tông / epoxy | "auto"
FLOOR_PROC_WIDTH = 160              # Độ phân giải xử lý (nhỏ = nhanh); 160x120 là đủ
FLOOR_BINS = 32                     # Số "tia" quét (cột ảnh)
FLOOR_MAX_RANGE_M = 3.0             # Tầm quét tối đa (m)
FLOOR_REF_ROWS = (0.84, 0.98)       # Vùng tham chiếu (sàn ngay trước robot) theo chiều cao ảnh
FLOOR_REF_COLS = (0.33, 0.67)       # ... theo chiều ngang ảnh
FLOOR_INIT_ROWS = (0.62, 0.98)      # Vùng học LẦN ĐẦU / khi nhấn F (lớn, phủ vài viên gạch) – phải trống
FLOOR_INIT_COLS = (0.12, 0.88)
FLOOR_BOTTOM_CROP = 0.0             # Tỉ lệ đáy ảnh bị thân robot che (bỏ qua)
FLOOR_LIKE_RATIO = 0.12             # Ngưỡng "giống sàn" (nhỏ = dễ coi là sàn, lớn = nhạy vật cản)
FLOOR_LEARN_RATE = 0.08             # Tốc độ thích nghi màu sàn khi đang chạy (đổi loại sàn)
FLOOR_MIN_OBST_HEIGHT_M = 0.08      # Chỉ vật NHÔ CAO hơn mức này mới là vật cản (bỏ vết bẩn, vạch sơn)
FLOOR_STOP_ENABLE = True            # Sàn bị chặn trong làn đường < OBST_STOP_DISTANCE_M -> dừng

# --- Bộ lái cục bộ (lách vật cản theo khe trống) ---
NAV_AVOID_ENABLE = True             # True: lách vật cản còn ở xa; vật cản trong làn gần hơn OBST_STOP_DISTANCE_M
                                    #       thì LUÔN DỪNG. False: không lách, chỉ dừng chờ.
ROBOT_HALF_WIDTH_M = 0.17           # 1/2 bề rộng robot kể cả bánh (ĐO LẠI!)
NAV_SAFETY_MARGIN_M = 0.05          # Khoảng an toàn thêm mỗi bên
NAV_LOOKAHEAD_M = 1.5               # Nhìn trước bao xa để chọn khe đi
NAV_SLOW_DISTANCE_M = 1.2           # Bắt đầu giảm tốc khi vật cản gần hơn khoảng này
NAV_AVOID_MODE = "strafe"           # "strafe": đi chéo/ngang tránh (mecanum, giữ camera nhìn đích) | "turn"
NAV_W_DIR = 0.6                     # Trọng số giữ hướng đích khi chọn khe
NAV_K_TURN = 1.2                    # Độ lợi quay về khe (chế độ turn)

# =====================================================================
# [G3] DẠY – LẶP LỘ TRÌNH BẰNG CAMERA (teach & repeat, khi trạm kế tiếp bị khuất)
# =====================================================================
ROUTE_ENABLE = True
ROUTE_DIR = "routes"                # Thư mục lưu lộ trình đã dạy (leg_<từ>_to_<đến>.npz)
ROUTE_KF_SPACING_M = 0.25           # Khoảng cách giữa 2 ảnh mốc khi dạy
ROUTE_KF_TURN_DEG = 12.0            # Hoặc quay quá góc này thì thêm ảnh mốc
ROUTE_FEATURES = 400                # Số đặc trưng ORB mỗi ảnh
ROUTE_FLOOR_EXCLUDE = 0.40          # Bỏ phần sàn dưới đáy ảnh khi lấy đặc trưng (ron gạch lặp lại)
ROUTE_MIN_MATCHES = 18              # Ít hơn số điểm khớp này -> coi như mất dấu
ROUTE_LOST_S = 2.5                  # Mất dấu lâu hơn -> chuyển sang quay tìm trạm
ROUTE_V = 0.18                      # Tốc độ tiến khi chạy theo lộ trình (m/s)
ROUTE_K_W = 1.2                     # Độ lợi quay theo độ lệch ảnh mốc
ROUTE_HANDOFF_DISTANCE_M = 2.5      # Thấy biển trạm gần hơn khoảng này -> chuyển sang tiếp cận bằng YOLO

# =====================================================================
# [H] LA BÀN (nối I2C của Raspberry Pi: SDA=GPIO2, SCL=GPIO3)
# =====================================================================
COMPASS_ENABLE = True
COMPASS_TYPE = "qmc5883l"           # "qmc5883l" (GY-271 đời mới, 0x0D) | "hmc5883l" (0x1E)
COMPASS_I2C_BUS = 1
COMPASS_CAL_FILE = "compass_cal.json"   # Tạo bằng: python tools/compass_calibrate.py

# =====================================================================
# [I] HIỂN THỊ & GIAO DIỆN VẬN HÀNH
# =====================================================================
DISPLAY_GUI = True                  # Màn hình HDMI/DSI gắn Pi (cửa sổ OpenCV)
DISPLAY_FULLSCREEN = False
DISPLAY_SIZE = (800, 480)           # Hợp màn hình 5"/7" phổ biến
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"   # Font có dấu tiếng Việt
LCD1602_ENABLE = False              # Thêm LCD 16x2 I2C (PCF8574) trên I2C của Pi
LCD1602_ADDR = 0x27

# =====================================================================
# [J] NÚT NHẤN / CÒI TRÊN RASPBERRY PI (tùy chọn)
# =====================================================================
GPIO_ENABLE = False
BTN_START_PIN = 17                  # Nút START/PAUSE (NO, nối GND)
BTN_STOP_PIN = 27                   # Nút STOP (NO, nối GND)
BUZZER_PIN = 18                     # Còi chíp 5V qua transistor

# =====================================================================
# [K] AN TOÀN PHẦN MỀM PHÍA PI
# =====================================================================
LINK_TIMEOUT_S = 1.0                # Không nhận dữ liệu Arduino -> lỗi mất kết nối
CAMERA_TIMEOUT_S = 2.0              # Không có khung hình mới -> lỗi camera
LOOP_MIN_PERIOD_S = 0.05            # Vòng lặp chính tối đa 20 Hz
LOG_DIR = "logs"
