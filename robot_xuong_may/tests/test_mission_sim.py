# -*- coding: utf-8 -*-
"""Kiểm thử tích hợp: Mission + bộ lái cục bộ + động học + Arduino giả + xưởng mô phỏng (đồng hồ ảo)."""
import math

from robot.kinematics import MecanumKinematics
from robot.mission import Mission, St
from robot.simworld import SimLink, SimRouteManager, SimWorld

from conftest import make_cfg


def run(cfg, seconds, obstacle_at=None, remove_obstacle_at=None, dt=0.1, floor=True,
        station_range=7.0, legs=None, boxes=(), seed=3):
    kin = MecanumKinematics.from_config(cfg)
    link = SimLink(cfg)
    world = SimWorld(cfg, kin, seed=seed, station_range=station_range)
    for b in boxes:
        world.add_box(*b)
    routes = None
    if legs is not None:
        routes = SimRouteManager(cfg, world)
        for (a, b), pts in legs.items():
            routes.set_leg(a, b, pts)
    m = Mission(cfg, routes=routes)
    t = 0.0
    m.update(t, [])
    m.start()
    arrivals, states = [], []
    obstacle_stops = 0
    min_clear = float("inf")
    route = None
    while t < seconds:
        if obstacle_at is not None and abs(t - obstacle_at) < dt / 2:
            world.toggle_obstacle_ahead(0.9)
        if remove_obstacle_at is not None and abs(t - remove_obstacle_at) < dt / 2:
            world.obstacles.clear()
        if routes is not None:
            if m.consume_route_reset():
                routes.start_leg(m.prev_station, m.target, None, 0.0)
            route = routes.follow(None, 0.0) if m.wants_route else None
        out = m.update(t, world.detections(), world.yaw_deg, {},
                       free=world.free_space() if floor else None, route=route,
                       pose=(world.x, world.y, world.th))
        if out.beep == "arrive":
            arrivals.append((round(t, 1), out.station, round(world.distance_to_station(out.station), 2)))
        if out.beep == "obstacle":
            obstacle_stops += 1
        states.append(out.state)
        if out.drive:
            link.send_wheels(kin.inverse(out.vx, out.vy, out.wz))
        else:
            link.stop()
        for _ in range(5):
            link.step(dt / 5)
            world.step(dt / 5, link.telemetry.rpm)
        min_clear = min(min_clear, world.clearance())
        t += dt
    return arrivals, states, obstacle_stops, world, min_clear


def test_full_route_visits_stations_in_order():
    cfg = make_cfg()
    arrivals, states, _, _, _ = run(cfg, 200.0)
    order = [a[1] for a in arrivals]
    assert order[:4] == [1, 2, 3, 1], arrivals
    for _, sid, dist in arrivals:
        # dừng gần khoảng cách đặt (chân trạm là vật cản thật nhưng không chặn lúc tiếp cận)
        assert 0.25 < dist < cfg.ARRIVE_DISTANCE_M + 0.15, arrivals
    times = [a[0] for a in arrivals]
    assert all(t2 - t1 > cfg.STATIONS[1]["wait_s"] for t1, t2 in zip(times, times[1:]))
    assert St.LOST not in states and St.OBSTACLE not in states


def test_full_route_without_floor_sensor():
    arrivals, _, _, _, _ = run(make_cfg(), 120.0, floor=False)
    assert [a[1] for a in arrivals][:3] == [1, 2, 3], arrivals


def test_strafe_mode_also_works():
    cfg = make_cfg(APPROACH_MODE="strafe")
    arrivals, _, _, _, _ = run(cfg, 120.0)
    assert [a[1] for a in arrivals][:3] == [1, 2, 3], arrivals


def test_obstacle_in_path_stops_robot_then_continues():
    cfg = make_cfg()
    arrivals, states, stops, world, _ = run(cfg, 120.0, obstacle_at=4.0, remove_obstacle_at=15.0)
    assert stops >= 1
    i4, i14 = int(4.5 / 0.1), int(14.5 / 0.1)
    assert St.OBSTACLE in states[i4:i14]
    assert [a[1] for a in arrivals][:2] == [1, 2], arrivals


def test_avoids_static_box_on_the_way_without_collision():
    """Thùng hàng nằm giữa đường tới TRẠM 1: robot lách qua (mecanum đi chéo) và vẫn tới trạm."""
    cfg = make_cfg()
    arrivals, states, _, world, min_clear = run(cfg, 60.0, boxes=[(1.6, -0.35, 0.18, 0.5)])
    assert arrivals and arrivals[0][1] == 1, arrivals
    assert min_clear > 0.0, min_clear                      # không va chạm
    assert min_clear > cfg.NAV_SAFETY_MARGIN_M * 0.5


def test_strict_stop_mode_waits_instead_of_avoiding():
    cfg = make_cfg(NAV_AVOID_ENABLE=False)
    arrivals, states, stops, world, min_clear = run(cfg, 40.0, boxes=[(1.6, -0.35, 0.18, 0.5)])
    assert St.OBSTACLE in states
    assert min_clear > 0.0
    assert not arrivals                                    # chỉ dừng chờ, không tự lách


def _legs():
    # lộ trình đã "dạy": các điểm đi qua (giả lập bằng tư thế thật)
    st = SimWorld.DEFAULT_STATIONS
    def line(a, b, n=12):
        return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]
    p0 = (0.0, 0.0)
    stop = lambda s, frm: (st[s][0] - 0.6 * (st[s][0] - frm[0]) / math.dist(st[s], frm),
                           st[s][1] - 0.6 * (st[s][1] - frm[1]) / math.dist(st[s], frm))
    s1 = stop(1, p0)
    s2 = stop(2, s1)
    s3 = stop(3, s2)
    return {(0, 1): line(p0, s1), (1, 2): line(s1, s2), (2, 3): line(s2, s3)}


def test_teach_and_repeat_reaches_hidden_stations():
    """Biển trạm chỉ thấy được trong 1.8 m (bị khuất): KHÔNG có lộ trình -> lạc,
    CÓ lộ trình đã dạy -> đi đủ TRẠM 1 -> 2 -> 3."""
    cfg = make_cfg(ROUTE_LOOP=False, SEARCH_TIMEOUT_S=15.0, LOST_AUTO_RETRY_S=0.0)
    arrivals, states, _, _, _ = run(cfg, 150.0, station_range=1.8, legs=_legs())
    assert [a[1] for a in arrivals] == [1, 2, 3], arrivals
    assert St.FOLLOW in states and St.APPROACH in states
    no_route, states2, _, _, _ = run(cfg, 60.0, station_range=1.8, legs=None)
    assert len(no_route) == 0 and St.LOST in states2
