# -*- coding: utf-8 -*-
"""Kiểm thử nhận diện sàn trống (gạch đá hoa, gạch bông, bê tông) và bộ lái cục bộ."""
import math

import numpy as np
import pytest

from robot.floor import FloorDetector, FreeSpace
from robot.geometry import CameraGeometry
from robot.navigator import LocalPlanner, ObstacleMemory

from conftest import make_cfg
from synth_floor import render

BROWN, BLUE, BLACK, RED = (40, 70, 120), (160, 90, 40), (30, 30, 30), (40, 40, 190)


def detector(kind_cfg="auto", tilt=0.0):
    cfg = make_cfg(FLOOR_TYPE=kind_cfg, CAMERA_TILT_DEG=tilt)
    return cfg, CameraGeometry.from_config(cfg)


def test_geometry_round_trip():
    g = CameraGeometry(62.2, 48.8, 0.25, 12.0)
    for X, Y in [(0.7, 0.0), (1.0, 0.2), (2.5, -0.6)]:
        x, y = g.image_of_ground(X, Y)
        assert g.ground_distance(y) == pytest.approx(X, rel=1e-6)
        assert g.lateral(x, X, y) == pytest.approx(Y, abs=1e-6)


@pytest.mark.parametrize("tilt", [0.0, 10.0])
@pytest.mark.parametrize("kind,ftype", [("tile", "tile"), ("tile", "auto"), ("pattern", "tile"),
                                        ("pattern", "auto"), ("concrete", "concrete"), ("concrete", "auto")])
def test_no_false_obstacles_on_clean_floor(kind, ftype, tilt):
    """Ron gạch, hoa văn gạch bông, vệt phản sáng, vết dầu, vết nứt, vạch sơn KHÔNG phải vật cản."""
    cfg, g = detector(ftype, tilt)
    fd = FloorDetector(cfg, g)
    fd.process(render(kind, g, seed=1))               # học sàn lần đầu (vùng lớn)
    fs = fd.process(render(kind, g, seed=7))          # ảnh khác (nhiễu khác)
    assert fs.valid and fs.learned
    X, _ = fs.points()
    assert X.size == 0, X


@pytest.mark.parametrize("tilt", [0.0, 10.0])
@pytest.mark.parametrize("kind", ["tile", "pattern", "concrete"])
@pytest.mark.parametrize("color", [BROWN, BLUE, BLACK, RED])
def test_box_detected_at_right_distance(kind, color, tilt):
    cfg, g = detector("auto", tilt)
    fd = FloorDetector(cfg, g)
    fd.process(render(kind, g, seed=1))
    fs = fd.process(render(kind, g, boxes=[(1.2, 0.0, 0.30, 0.40, color)], seed=2))
    d = fs.nearest_in_lane(0.22)
    assert 1.0 <= d <= 1.3, d                         # thật: 1.2 m (sai số nghiêng về an toàn)
    X, Y = fs.points()
    assert np.all(np.abs(Y) < 0.35)                   # chỉ báo đúng chỗ hộp, không lan sang bên


def test_box_side_and_lateral_sign():
    cfg, g = detector("auto")
    fd = FloorDetector(cfg, g)
    fd.process(render("tile", g, seed=1))
    fs = fd.process(render("tile", g, boxes=[(1.5, 0.5, 0.3, 0.4, RED)], seed=2))
    X, Y = fs.points()
    assert X.size > 0 and np.all(Y > 0.25)            # hộp lệch TRÁI 0.5 m -> Y dương
    assert fs.nearest_in_lane(0.22) == math.inf       # không nằm trong làn robot


def test_learn_now_resets_model():
    cfg, g = detector("auto")
    fd = FloorDetector(cfg, g)
    fd.process(render("concrete", g, seed=1))
    fd.learn_now()
    assert not fd.ready
    fs = fd.process(render("tile", g, seed=1))        # đổi loại sàn + học lại
    assert fs.valid
    assert fs.points()[0].size == 0


def _fs(points):
    X = np.array([p[0] for p in points], float)
    Y = np.array([p[1] for p in points], float)
    return FreeSpace(valid=True, dist=X, lat=Y, max_range=10.0)


def test_planner_clear_keeps_command():
    p = LocalPlanner(make_cfg())
    r = p.adjust(0.2, 0.0, -0.1, _fs([(2.8, 1.0)]), 0.0, 1.5, 0.8)
    assert r.status == "clear" and r.vx == pytest.approx(0.2) and r.wz == -0.1


def test_planner_strafes_away_from_obstacle():
    p = LocalPlanner(make_cfg(NAV_AVOID_MODE="strafe"))
    wall = [(1.2, y) for y in np.linspace(-0.4, 0.05, 10)]   # vật cản lệch PHẢI
    r = p.adjust(0.2, 0.0, 0.0, _fs(wall), 0.0, 1.5, 0.8)
    assert r.status == "avoid" and r.vy > 0 and r.theta > 0   # đi chéo sang TRÁI


def test_planner_turn_mode():
    p = LocalPlanner(make_cfg(NAV_AVOID_MODE="turn"))
    wall = [(1.2, y) for y in np.linspace(-0.05, 0.4, 10)]   # vật cản lệch TRÁI
    r = p.adjust(0.2, 0.0, 0.0, _fs(wall), 0.0, 1.5, 0.8)
    assert r.status == "avoid" and r.wz < 0                   # quay PHẢI


def test_planner_blocked_when_no_gap():
    p = LocalPlanner(make_cfg())
    wall = [(0.9, y) for y in np.linspace(-1.5, 1.5, 60)]
    r = p.adjust(0.2, 0.0, 0.0, _fs(wall), 0.0, 1.5, 0.8)
    assert r.status == "blocked" and r.vx == 0 and r.vy == 0


def test_planner_allows_rotation_in_place():
    p = LocalPlanner(make_cfg())
    wall = [(0.6, y) for y in np.linspace(-1.5, 1.5, 60)]
    r = p.adjust(0.0, 0.0, 0.8, _fs(wall), 0.0, 1.5, 0.8)
    assert r.status == "clear" and r.wz == 0.8


def test_obstacle_memory_keeps_points_beside_robot():
    cfg = make_cfg()
    g = CameraGeometry.from_config(cfg)
    mem = ObstacleMemory(g)
    mem.update(np.array([1.0]), np.array([0.0]), (0.0, 0.0, 0.0), 0.0)
    # robot tiến 0.9 m và lệch trái 0.3 m: vật giờ ở sát bên phải, ngoài tầm nhìn
    X, Y = mem.recall((0.9, 0.3, 0.0))
    assert X.size > 0 and np.any((X > -0.1) & (X < 0.5) & (Y < -0.2))
    # quay lại nhìn thẳng vào chỗ cũ mà vật đã đi -> quên (quan sát mới thay thế)
    mem.update(np.zeros(0), np.zeros(0), (0.0, 0.0, 0.0), 1.0)
    assert mem.recall((0.0, 0.0, 0.0))[0].size == 0
    # hết thời gian nhớ
    mem.update(np.array([0.3]), np.array([0.4]), (0.0, 0.0, 0.0), 2.0)
    mem.update(np.zeros(0), np.zeros(0), (0.0, 0.0, 0.0), 20.0)
    assert mem.recall((0.0, 0.0, 0.0))[0].size == 0
