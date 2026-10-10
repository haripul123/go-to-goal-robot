import math

import pytest

from robot import GOAL_TOLERANCE, go_to_goal, step, wrap_angle


def test_wrap_angle_keeps_small_angles():
    assert wrap_angle(0.5) == pytest.approx(0.5)


def test_wrap_angle_wraps_large_angles():
    # 270 degrees is the same direction as -90 degrees
    assert wrap_angle(math.radians(270)) == pytest.approx(math.radians(-90))


def test_step_moves_forward_along_x():
    x, y, theta = step(0.0, 0.0, 0.0, v=1.0, omega=0.0, dt=1.0)
    assert (x, y, theta) == pytest.approx((1.0, 0.0, 0.0))


@pytest.mark.parametrize(
    "start, goal",
    [
        ((0.0, 0.0, math.pi / 2), (3.0, 2.0)),   # facing up
        ((0.0, 0.0, math.pi), (2.0, 0.0)),       # facing away from goal
        ((1.0, 1.0, 0.0), (-2.0, -3.0)),         # goal behind and below
    ],
)
def test_robot_reaches_goal(start, goal):
    path = go_to_goal(*start, *goal)
    assert math.dist(path[-1], goal) < GOAL_TOLERANCE


def test_very_low_turn_gain_fails():
    path = go_to_goal(0.0, 0.0, math.pi / 2, 3.0, 2.0, k_turn=0.3, max_steps=600)
    assert math.dist(path[-1], (3.0, 2.0)) > GOAL_TOLERANCE