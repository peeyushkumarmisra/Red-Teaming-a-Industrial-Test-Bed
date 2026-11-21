# =============================================================================
#  test_trajectory.py - smooth moves
# =============================================================================

import numpy as np
import pytest

from ids_experiments.trajectory import move_duration, quintic

Q0 = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])
Q1 = np.array([1.0, 0.2, -0.5, -1.8, 0.3, 0.5, 0.0])


# No jump in speed at the start or end of a move.
def test_quintic_starts_and_ends_at_rest():
    p, v, a = quintic(Q0, Q1, 2.0, 0.0)
    np.testing.assert_allclose(p, Q0)
    np.testing.assert_allclose(v, 0.0)
    np.testing.assert_allclose(a, 0.0)
    p, v, a = quintic(Q0, Q1, 2.0, 2.0)
    np.testing.assert_allclose(p, Q1)
    np.testing.assert_allclose(v, 0.0)
    np.testing.assert_allclose(a, 0.0)


# Reported velocity and acceleration are consistent.
def test_quintic_velocity_and_acceleration_are_derivatives_of_position():
    h = 1e-6
    for t in (0.3, 1.0, 1.7):
        p_plus, v_plus, _ = quintic(Q0, Q1, 2.0, t + h)
        p_minus, v_minus, _ = quintic(Q0, Q1, 2.0, t - h)
        _, v, a = quintic(Q0, Q1, 2.0, t)
        np.testing.assert_allclose((p_plus - p_minus) / (2 * h), v, atol=1e-6)
        np.testing.assert_allclose((v_plus - v_minus) / (2 * h), a, atol=1e-5)


def test_quintic_midpoint_is_half_way():
    p, _, a = quintic(Q0, Q1, 4.0, 2.0)
    np.testing.assert_allclose(p, 0.5 * (Q0 + Q1))
    np.testing.assert_allclose(a, 0.0, atol=1e-12)


# Moves never exceed the joint limits.
def test_move_duration_respects_velocity_and_acceleration_limits():
    vmax = np.full(7, 0.5)
    amax = np.full(7, 1.0)
    duration = move_duration(Q0, Q1, vmax, amax)
    peak_v = 0.0
    peak_a = 0.0
    for t in np.linspace(0.0, duration, 2001):
        _, v, a = quintic(Q0, Q1, duration, t)
        peak_v = max(peak_v, np.abs(v).max())
        peak_a = max(peak_a, np.abs(a).max())
    assert peak_v <= 0.5 + 1e-6
    assert peak_a <= 1.0 + 1e-3
    assert max(peak_v / 0.5, peak_a / 1.0) == pytest.approx(1.0, abs=1e-3)


def test_move_duration_keeps_a_minimum():
    assert move_duration(Q0, Q0, np.ones(7), np.ones(7), minimum=2.0) == 2.0
