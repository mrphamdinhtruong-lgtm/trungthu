# -*- coding: utf-8 -*-
"""Kiểm thử dạy – lặp lộ trình bằng ảnh mốc ORB trên ảnh tổng hợp (dịch ngang = quay, phóng to = tiến)."""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from robot.route_memory import FeatureExtractor, RouteFollower, RouteManager, RouteMemory, RouteRecorder

from conftest import make_cfg


def scene(seed=0, W=1600, H=700):
    """Ảnh "toàn cảnh" xưởng: nhiều khối, chữ, đường kẻ (máy may, kệ, cột...)."""
    rng = np.random.default_rng(seed)
    img = np.full((H, W, 3), 120, np.uint8)
    for _ in range(260):
        x, y = int(rng.integers(0, W)), int(rng.integers(0, H))
        w, h = int(rng.integers(10, 90)), int(rng.integers(10, 90))
        col = tuple(int(c) for c in rng.integers(0, 255, 3))
        if rng.random() < 0.5:
            cv2.rectangle(img, (x, y), (x + w, y + h), col, -1)
        else:
            cv2.circle(img, (x, y), w // 2, col, -1)
    for i in range(40):
        cv2.putText(img, f"M{i}", (int(rng.integers(0, W)), int(rng.integers(20, H))),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return img


def view(sc, shift=0, zoom=1.0, W=640, H=480):
    """Góc nhìn camera: shift > 0 -> nhìn lệch PHẢI (robot quay phải); zoom > 1 -> tiến lại gần."""
    cy, cx = sc.shape[0] // 2, sc.shape[1] // 2 + shift
    w, h = int(W / zoom), int(H / zoom)
    crop = sc[cy - h // 2: cy + h // 2, cx - w // 2: cx + w // 2]
    return cv2.resize(crop, (W, H))


@pytest.fixture(scope="module")
def sc():
    return scene()


def test_steering_sign(sc):
    cfg = make_cfg()
    ex = FeatureExtractor(320, 400, 0.0)
    mem = RouteMemory(0, 1)
    rec = RouteRecorder(0, 1, ex)
    rec.feed(view(sc, 0), 0.0, 0.0)
    rec.feed(view(sc, 0, 1.02), 0.3, 0.0)
    mem = rec.finish(0.3, 0.0)
    f = RouteFollower(mem, ex, cfg)
    f.start(view(sc, 0), 0.0)
    # robot đang lệch TRÁI so với lúc dạy (nhìn lệch trái = shift âm) -> cảnh dịch phải -> phải quay PHẢI
    g = f.update(view(sc, -40), 0.0, now=0.0)
    assert g.ok and g.ex > 0 and g.wz < 0
    g = f.update(view(sc, 40), 0.0, now=0.1)
    assert g.ok and g.ex < 0 and g.wz > 0
    g = f.update(view(sc, 0), 0.0, now=0.2)
    assert abs(g.ex) < 0.05


def test_progress_through_keyframes_and_done(sc):
    cfg = make_cfg()
    ex = FeatureExtractor(320, 400, 0.0)
    rec = RouteRecorder(0, 1, ex, spacing_m=0.25)
    zooms = [1.0 + 0.08 * k for k in range(8)]
    for k, z in enumerate(zooms):
        rec.feed(view(sc, 0, z), 0.25 * k, 0.0)
    mem = rec.finish(0.25 * 7, 0.0)
    assert len(mem.keyframes) >= 8
    f = RouteFollower(mem, ex, cfg)
    f.start(view(sc, 0, 1.0), 0.0)
    idx = []
    for k, z in enumerate(zooms):
        g = f.update(view(sc, 0, z), 0.25 * k, now=k * 0.1)
        idx.append(g.idx)
    assert idx == sorted(idx) and idx[-1] >= len(mem.keyframes) - 2
    g = f.update(view(sc, 0, zooms[-1]), 0.25 * 7, now=1.0)
    g = f.update(view(sc, 0, zooms[-1]), 0.25 * 7, now=1.1)
    assert g.done


def test_relocalize_in_middle_and_lost(sc):
    cfg = make_cfg(ROUTE_LOST_S=0.5)
    ex = FeatureExtractor(320, 400, 0.0)
    rec = RouteRecorder(0, 1, ex)
    for k in range(6):
        rec.feed(view(sc, 0, 1.0 + 0.1 * k), 0.25 * k, 0.0)
    mem = rec.finish(1.25, 0.0)
    f = RouteFollower(mem, ex, cfg)
    start = f.start(view(sc, 0, 1.3), 1.0)            # chạy tiếp từ giữa chặng
    assert 2 <= start <= 4
    blank = np.full((480, 640, 3), 128, np.uint8)     # che camera / tường trơn
    g = f.update(blank, 1.0, now=0.0)
    assert not g.ok and not g.lost and g.vx == 0
    g = f.update(blank, 1.0, now=1.0)
    assert g.lost


def test_manager_save_load_round_trip(tmp_path, sc):
    cfg = make_cfg(ROUTE_DIR=str(tmp_path))
    rm = RouteManager(cfg)
    assert not rm.has(1, 2)
    rm.start_teach(1, 2)
    for k in range(5):
        rm.teach_feed(view(sc, 0, 1.0 + 0.08 * k), 0.26 * k, 0.0)
    n, dist = rm.teach_info()
    assert n == 5 and dist == pytest.approx(1.04)
    mem = rm.stop_teach(True, 1.1, 0.0)
    assert mem is not None and rm.has(1, 2) and rm.legs() == [(1, 2)]
    back = RouteMemory.load(rm.path(1, 2))
    assert len(back.keyframes) == len(mem.keyframes)
    assert np.array_equal(back.keyframes[2].desc, mem.keyframes[2].desc)
    assert rm.start_leg(1, 2, view(sc, 0, 1.0), 0.0)
    g = rm.follow(view(sc, 0, 1.0), 0.0)
    assert g is not None and g.ok
