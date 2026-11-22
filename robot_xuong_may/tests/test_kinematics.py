# -*- coding: utf-8 -*-
import math

import pytest

from robot.kinematics import RADS_TO_RPM, MecanumKinematics


@pytest.fixture
def kin():
    return MecanumKinematics(0.0485, 0.10, 0.115, rpm_limit=85.0)


def test_forward_all_wheels_equal(kin):
    rpm = kin.inverse(0.2, 0.0, 0.0)
    assert all(abs(x - rpm[0]) < 1e-9 for x in rpm)
    assert rpm[0] == pytest.approx(0.2 / 0.0485 * RADS_TO_RPM)


def test_strafe_left_pattern(kin):
    fl, fr, rl, rr = kin.inverse(0.0, 0.2, 0.0)
    # đi ngang trái (con lăn chữ X): FL lùi, FR tiến, RL tiến, RR lùi
    assert fl < 0 and fr > 0 and rl > 0 and rr < 0


def test_rotate_ccw_pattern(kin):
    fl, fr, rl, rr = kin.inverse(0.0, 0.0, 1.0)
    assert fl < 0 and rl < 0 and fr > 0 and rr > 0


@pytest.mark.parametrize("v", [(0.1, 0.05, 0.3), (-0.15, 0.1, -0.5), (0.0, -0.12, 0.2)])
def test_round_trip(kin, v):
    back = kin.forward(kin.inverse(*v))
    assert back == pytest.approx(v, abs=1e-9)


def test_saturation_keeps_direction(kin):
    rpm = kin.inverse(1.0, 0.5, 0.0)      # vượt giới hạn
    assert max(abs(x) for x in rpm) == pytest.approx(85.0)
    vx, vy, wz = kin.forward(rpm)
    assert vy / vx == pytest.approx(0.5)
    assert abs(wz) < 1e-9


def test_max_speed(kin):
    assert kin.max_speed() == pytest.approx(85.0 / 60.0 * 2 * math.pi * 0.0485)
