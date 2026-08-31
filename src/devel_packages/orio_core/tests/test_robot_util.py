"""Unit tests for orio_core.robot_util (the already-pure kinematics/geometry lib)."""
import math

import numpy as np

from orio_core import robot_util as ru


def test_rpyxyz2H_pure_translation():
    H = ru.rpyxyz2H([0.0, 0.0, 0.0], [1.0, 2.0, 3.0])
    assert np.allclose(H[:3, 3], [1.0, 2.0, 3.0])
    assert np.allclose(H[:3, :3], np.eye(3))
    assert np.allclose(H[3, :], [0, 0, 0, 1])


def test_rpyxyz2H_yaw_90deg():
    H = ru.rpyxyz2H([0.0, 0.0, math.pi / 2], [0.0, 0.0, 0.0])
    # +90deg about Z maps x-axis -> y-axis
    assert np.allclose(H[:3, 0], [0, 1, 0], atol=1e-9)
    assert np.allclose(H[:3, 1], [-1, 0, 0], atol=1e-9)


def test_R2axisang_identity_is_zero_angle():
    axis, ang = ru.R2axisang(np.eye(3))
    assert ang == 0.0
    assert list(axis) == [1, 0, 0]


def test_R2axisang_recovers_known_rotation():
    theta = 0.7
    Rz = np.array([[math.cos(theta), -math.sin(theta), 0],
                   [math.sin(theta),  math.cos(theta), 0],
                   [0, 0, 1]])
    axis, ang = ru.R2axisang(Rz)
    assert math.isclose(ang, theta, abs_tol=1e-9)
    assert np.allclose(axis, [0, 0, 1], atol=1e-9)


def test_interp_min_jerk_endpoints():
    q0 = np.array([0.0, 1.0, -2.0])
    q1 = np.array([1.0, -1.0, 0.5])
    T = 2.0
    # At t=0: at start, zero velocity
    q_s, qd_s = ru.interp_min_jerk(q0, q1, 0.0, T)
    assert np.allclose(q_s, q0)
    assert np.allclose(qd_s, 0.0)
    # At t=T: at goal, zero velocity
    q_e, qd_e = ru.interp_min_jerk(q0, q1, T, T)
    assert np.allclose(q_e, q1)
    assert np.allclose(qd_e, 0.0, atol=1e-9)


def test_interp_min_jerk_midpoint_is_halfway():
    q0 = np.zeros(3)
    q1 = np.ones(3)
    q_mid, _ = ru.interp_min_jerk(q0, q1, 1.0, 2.0)  # s(0.5) = 0.5 for min-jerk
    assert np.allclose(q_mid, 0.5)


def test_axis_angle_between_aligned_and_opposed():
    axis, ang = ru.axis_angle_between(np.array([1.0, 0, 0]), np.array([2.0, 0, 0]))
    assert math.isclose(ang, 0.0, abs_tol=1e-9)
    axis, ang = ru.axis_angle_between(np.array([1.0, 0, 0]), np.array([-1.0, 0, 0]))
    assert math.isclose(ang, math.pi, abs_tol=1e-9)
