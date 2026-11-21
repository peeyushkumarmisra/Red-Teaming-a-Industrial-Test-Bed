# =============================================================================
#  test_attacks.py - attack shapes and ground truth
# =============================================================================

import numpy as np
import pytest

from ids_experiments.attacks import Attack, AttackSchedule

Q = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])
V = np.full(7, 0.1)


# Outside an attack the MitM changes nothing.
def test_transparent_outside_the_attack_window():
    s = AttackSchedule([Attack(1, 'step', 3, 0.02, 10.0, 5.0)])
    for t in (0.0, 9.999, 15.0, 20.0):
        q, v, truth = s.apply(t, Q, V)
        np.testing.assert_array_equal(q, Q)
        np.testing.assert_array_equal(v, V)
        assert not truth.attack_active
        assert truth.event_id == 0
        assert not any(truth.joint_mask)


# A step changes one joint's position only.
def test_step_offsets_only_the_target_joint_position():
    s = AttackSchedule([Attack(7, 'step', 3, 0.02, 10.0, 5.0)])
    q, v, truth = s.apply(12.0, Q, V)
    expected = Q.copy()
    expected[3] += 0.02
    np.testing.assert_allclose(q, expected)
    np.testing.assert_array_equal(v, V)
    assert truth.attack_active
    assert truth.kind == 'step'
    assert truth.joint_mask == (False, False, False, True, False, False, False)
    assert truth.event_id == 7
    assert truth.magnitude == pytest.approx(0.02)


# A ramp grows linearly and changes the velocity setpoint too.
def test_ramp_grows_with_time_and_shifts_the_velocity():
    s = AttackSchedule([Attack(2, 'ramp', 1, 0.001, 10.0, 20.0)])
    q, v, _ = s.apply(14.0, Q, V)
    assert q[1] - Q[1] == pytest.approx(0.004)
    assert v[1] - V[1] == pytest.approx(0.001)
    assert np.count_nonzero(q - Q) == 1


def test_inputs_are_not_modified():
    q_in = Q.copy()
    AttackSchedule([Attack(1, 'step', 0, 0.1, 0.0, 1.0)]).apply(0.5, q_in, V)
    np.testing.assert_array_equal(q_in, Q)


def test_events_are_selected_by_time():
    s = AttackSchedule([Attack(2, 'ramp', 5, 0.001, 30.0, 5.0),
                        Attack(1, 'step', 0, 0.02, 10.0, 5.0)])
    assert s.apply(12.0, Q, V)[2].event_id == 1
    assert s.apply(32.0, Q, V)[2].event_id == 2
    assert s.apply(20.0, Q, V)[2].event_id == 0
