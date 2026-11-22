# -*- coding: utf-8 -*-
"""Kiểm thử máy trạng thái nhiệm vụ với nhận diện giả (không cần phần cứng)."""
from robot.common import Detection
from robot.mission import Mission, St

from conftest import make_cfg


def station_det(cls="tram_1", cx=0.5, h=0.2, conf=0.9):
    return Detection(cls, conf, cx - h * 0.4, 0.5 - h / 2, cx + h * 0.4, 0.5 + h / 2)


def obstacle_det(y2=0.98, cx=0.5, cls="nguoi"):
    return Detection(cls, 0.9, cx - 0.1, 0.2, cx + 0.1, y2)


class Clock:
    def __init__(self):
        self.t = 0.0

    def tick(self, dt=0.1):
        self.t += dt
        return self.t


def test_idle_until_start(cfg):
    m = Mission(cfg)
    out = m.update(0.0, [station_det()])
    assert out.state == St.IDLE and not out.drive
    assert "SẴN SÀNG" in out.title


def test_search_confirm_approach_arrive_wait_next(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    out = m.update(c.tick(), [])
    assert out.state == St.SEARCH and out.drive and out.wz != 0
    assert out.title == "ĐANG DI CHUYỂN ĐẾN TRẠM 1"
    # biển báo trạm KHÁC không được tính
    for _ in range(5):
        out = m.update(c.tick(), [station_det("tram_2")])
    assert out.state == St.SEARCH
    # thấy trạm 1 đủ số khung -> tiếp cận
    for _ in range(cfg.CONFIRM_FRAMES):
        out = m.update(c.tick(), [station_det("tram_1", cx=0.7, h=0.1)])
    assert out.state == St.APPROACH
    assert out.wz < 0            # trạm lệch phải -> quay phải (CW)
    assert out.distance > 1.0
    # tiến gần: chiều cao khung tăng dần
    h = 0.1
    while out.state == St.APPROACH and h < 0.9:
        h += 0.03
        out = m.update(c.tick(), [station_det("tram_1", cx=0.5, h=h)])
    assert out.state == St.WAIT
    assert out.beep == "arrive"
    assert out.title == "ĐÃ ĐẾN TRẠM 1" and not out.drive
    assert m.distance_from_det(station_det(h=h)) <= cfg.ARRIVE_DISTANCE_M
    # chờ đúng thời gian cấu hình (kế hoạch: 15 s)
    t_arrive = c.t
    while out.state == St.WAIT:
        out = m.update(c.tick(), [])
    assert abs((c.t - t_arrive) - cfg.STATIONS[1]["wait_s"]) < 0.25
    assert out.state == St.SEARCH and out.station == 2
    assert out.title == "ĐANG DI CHUYỂN ĐẾN TRẠM 2"


def test_countdown_text(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    for _ in range(cfg.CONFIRM_FRAMES + cfg.ARRIVE_FRAMES + 2):
        out = m.update(c.tick(), [station_det("tram_1", h=0.6)])
    assert out.state == St.WAIT
    out = m.update(c.tick(5.0), [])
    w = cfg.STATIONS[1]["wait_s"] - 5.0
    assert "còn" in out.subtitle and out.countdown is not None and w - 1 < out.countdown < w + 1


def test_obstacle_stops_and_resumes(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    for _ in range(cfg.CONFIRM_FRAMES):
        out = m.update(c.tick(), [station_det("tram_1", h=0.1)])
    assert out.state == St.APPROACH and out.vx > 0
    # 1 khung vật cản chưa đủ -> vẫn chạy (chống báo giả)
    out = m.update(c.tick(), [station_det("tram_1", h=0.1), obstacle_det()])
    assert out.state == St.APPROACH
    out = m.update(c.tick(), [station_det("tram_1", h=0.1), obstacle_det()])
    assert out.state == St.OBSTACLE and not out.drive and out.vx == 0
    assert out.title == "GẶP VẬT CẢN – ROBOT DỪNG" and out.beep == "obstacle"
    # vật cản còn đó -> vẫn dừng
    for _ in range(30):
        out = m.update(c.tick(), [station_det("tram_1", h=0.1), obstacle_det()])
    assert out.state == St.OBSTACLE
    # hết vật cản -> chờ OBST_CLEAR_S rồi chạy tiếp
    t_clear = c.t
    while out.state == St.OBSTACLE:
        out = m.update(c.tick(), [station_det("tram_1", h=0.1)])
    assert c.t - t_clear >= cfg.OBST_CLEAR_S - 1e-6
    assert out.state == St.APPROACH and out.vx > 0


def test_far_obstacle_and_side_obstacle_ignored(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    for _ in range(cfg.CONFIRM_FRAMES):
        m.update(c.tick(), [station_det("tram_1", h=0.1)])
    far = obstacle_det(y2=0.62)            # chạm sàn ở xa (~2.3 m)
    side = obstacle_det(y2=0.98, cx=0.05)  # sát mép trái, ngoài hành lang
    assert m.ground_distance(0.62) > cfg.OBST_STOP_DISTANCE_M
    for _ in range(10):
        out = m.update(c.tick(), [station_det("tram_1", h=0.1), far, side])
    assert out.state == St.APPROACH


def test_lost_target_returns_to_search(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    for _ in range(cfg.CONFIRM_FRAMES):
        out = m.update(c.tick(), [station_det("tram_1", cx=0.8, h=0.1)])
    assert out.state == St.APPROACH
    out = m.update(c.tick(), [])
    assert out.state == St.APPROACH            # mất dấu ngắn: vẫn giữ
    while out.state == St.APPROACH:
        out = m.update(c.tick(), [])
    assert out.state == St.SEARCH
    out = m.update(c.tick(), [])
    assert out.wz < 0                          # lần cuối thấy ở bên phải -> quay phải


def test_search_timeout_lost_and_retry(cfg):
    cfg = make_cfg(SEARCH_TIMEOUT_S=5.0, LOST_AUTO_RETRY_S=3.0)
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    out = None
    for _ in range(60):
        out = m.update(c.tick(), [])
    assert out.state == St.LOST and not out.drive
    assert "KHÔNG TÌM THẤY TRẠM 1" == out.title
    for _ in range(31):
        out = m.update(c.tick(), [])
    assert out.state == St.SEARCH


def test_estop_latched_until_reset(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    m.trigger_estop("Nút dừng khẩn")
    out = m.update(c.tick(), [station_det()])
    assert out.state == St.ESTOP and not out.drive and out.title == "DỪNG KHẨN CẤP"
    m.start()                                  # START không có tác dụng khi đang dừng khẩn
    assert m.update(c.tick(), []).state == St.ESTOP
    m.reset()
    assert m.update(c.tick(), []).state == St.IDLE


def test_fault_stops_and_recovers_to_idle(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    out = m.update(c.tick(), [], faults={"link": "Mất kết nối Arduino"})
    assert out.state == St.FAULT and not out.drive and "Mất kết nối Arduino" in out.subtitle
    out = m.update(c.tick(), [])
    assert out.state == St.IDLE and "khắc phục" in out.subtitle


def test_manual_blocks_forward_on_obstacle(cfg):
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.toggle_manual()
    m.set_manual(0.2, 0.0, 0.3, c.tick())
    out = m.update(c.t, [])
    assert out.state == St.MANUAL and out.vx == 0.2 and out.wz == 0.3
    for _ in range(cfg.OBST_ON_FRAMES):
        m.set_manual(0.2, 0.0, 0.0, c.tick())
        out = m.update(c.t, [obstacle_det()])
    assert out.vx == 0.0
    m.set_manual(-0.1, 0.0, 0.0, c.tick())
    out = m.update(c.t, [obstacle_det()])
    assert out.vx == -0.1                      # cho phép lùi tránh vật cản
    out = m.update(c.tick(1.0), [])
    assert out.vx == 0.0                       # nhả phím -> tự dừng


def test_route_no_loop_done(cfg):
    cfg = make_cfg(ROUTE=(1,), ROUTE_LOOP=False)
    m, c = Mission(cfg), Clock()
    m.update(c.t, [])
    m.start()
    for _ in range(cfg.CONFIRM_FRAMES + cfg.ARRIVE_FRAMES + 2):
        out = m.update(c.tick(), [station_det("tram_1", h=0.6)])
    m.skip_wait()
    out = m.update(c.tick(), [])
    assert out.state == St.DONE


def test_heading_hint_aligns_first(cfg):
    cfg = make_cfg()
    cfg.STATIONS[1]["heading_deg"] = 90.0
    m, c = Mission(cfg), Clock()
    m.update(c.t, [], yaw_deg=0.0)
    m.start()
    out = m.update(c.tick(), [], yaw_deg=0.0)
    assert out.wz > 0 and "la bàn" in out.subtitle      # cần quay trái 90°
    out = m.update(c.tick(), [], yaw_deg=88.0)
    out = m.update(c.tick(), [], yaw_deg=88.0)
    assert out.wz == -cfg.W_SEARCH                       # đã đúng hướng -> quét bình thường
