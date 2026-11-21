import math

import numpy as np
import pytest

from ids_attacks.library import KINDS, AttackSchedule, make_attack

Q = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])
V = np.full(7, 0.1)
A = np.full(7, 0.01)


def trajectory(t):
    q = Q + 0.2 * np.sin(0.5 * t)
    v = np.full(7, 0.1 * math.cos(0.5 * t))
    a = np.full(7, -0.05 * math.sin(0.5 * t))
    return q, v, a


def run(schedule, times):
    out = []
    for t in times:
        q, v, a = trajectory(t)
        out.append((t, q, v, a) + schedule.apply(t, q, v, a))
    return out


def test_every_kind_is_transparent_outside_its_window():
    for kind in KINDS:
        joints = (1, 3) if kind == 'coordinated' else (3,)
        attack = make_attack(1, kind, joints, 0.02 if kind != 'scaling' else 1.1, 10.0, 5.0,
                             frequency=1.0, lag=2.0)
        schedule = AttackSchedule([attack])
        for t, q, v, a, q2, v2, a2, truth in run(schedule, np.arange(0.0, 10.0, 0.01)):
            np.testing.assert_array_equal(q2, q)
            np.testing.assert_array_equal(v2, v)
            assert not truth.attack_active
        for t, q, v, a, q2, v2, a2, truth in run(schedule, np.arange(15.0, 16.0, 0.01)):
            np.testing.assert_array_equal(q2, q)
            assert not truth.attack_active


def test_step_and_coordinated_offset_only_their_joints():
    s = AttackSchedule([make_attack(7, 'step', (3,), 0.02, 10.0, 5.0)])
    q2, v2, _, truth = s.apply(12.0, Q, V, A)
    assert q2[3] - Q[3] == pytest.approx(0.02)
    assert np.count_nonzero(q2 - Q) == 1
    np.testing.assert_array_equal(v2, V)
    assert truth.joint_mask == (False, False, False, True, False, False, False)
    assert truth.event_id == 7 and truth.kind == 'step'
    c = AttackSchedule([make_attack(1, 'coordinated', (1, 3), start=0.0, duration=1.0,
                                    magnitudes=(0.004, -0.006))])
    q2, _, _, truth = c.apply(0.5, Q, V, A)
    assert q2[1] - Q[1] == pytest.approx(0.004)
    assert q2[3] - Q[3] == pytest.approx(-0.006)
    assert truth.magnitude == pytest.approx(0.006)
    assert truth.joint_mask == (False, True, False, True, False, False, False)


def test_ramp_and_sine_are_consistent_in_position_and_velocity():
    r = AttackSchedule([make_attack(1, 'ramp', (2,), 0.001, 10.0, 20.0)])
    q2, v2, _, _ = r.apply(14.0, Q, V, A)
    assert q2[2] - Q[2] == pytest.approx(0.004)
    assert v2[2] - V[2] == pytest.approx(0.001)
    s = AttackSchedule([make_attack(1, 'sine', (4,), 0.01, 0.0, 10.0, frequency=0.5)])
    dt = 1e-4
    q_a, v_a, _, _ = s.apply(1.3, Q, V, A)
    q_b, _, _, _ = s.apply(1.3 + dt, Q, V, A)
    assert (q_b[4] - q_a[4]) / dt == pytest.approx(v_a[4] - V[4], rel=1e-3)
    assert q_a[4] - Q[4] == pytest.approx(0.01 * math.sin(2 * math.pi * 0.5 * 1.3))


def test_scaling_amplifies_motion_from_the_attack_start():
    s = AttackSchedule([make_attack(1, 'scaling', (0,), 1.5, 2.0, 5.0)])
    rows = run(s, np.arange(2.0, 4.0, 0.01))
    q_start = rows[0][1][0]
    for t, q, v, a, q2, v2, a2, truth in rows:
        assert q2[0] - q_start == pytest.approx(1.5 * (q[0] - q_start))
        assert v2[0] == pytest.approx(1.5 * v[0])
        assert q2[1] == q[1]


def test_freeze_holds_the_sample_at_the_attack_start():
    s = AttackSchedule([make_attack(1, 'freeze', (5,), 0.0, 1.0, 2.0)])
    rows = run(s, np.arange(1.0, 2.5, 0.01))
    frozen = rows[0][1][5]
    for t, q, v, a, q2, v2, a2, truth in rows:
        assert q2[5] == frozen
        assert v2[5] == 0.0 and a2[5] == 0.0


def test_replay_loops_the_window_before_the_attack():
    s = AttackSchedule([make_attack(1, 'replay', (3,), 0.0, 5.0, 4.0, lag=1.0)])
    rows = {round(t, 2): (q, q2) for t, q, v, a, q2, v2, a2, truth in
            run(s, np.arange(0.0, 9.0, 0.01))}
    for t in (5.0, 5.37, 6.0, 7.5, 8.99):
        recorded = 4.0 + math.fmod(t - 5.0, 1.0)
        assert rows[round(t, 2)][1][3] == pytest.approx(rows[round(recorded, 2)][0][3], abs=1e-12)


def test_delay_forwards_the_sample_from_lag_seconds_earlier():
    s = AttackSchedule([make_attack(1, 'delay', (6,), 0.0, 5.0, 4.0, lag=0.3)])
    rows = {round(t, 2): (q, q2) for t, q, v, a, q2, v2, a2, truth in
            run(s, np.arange(0.0, 9.0, 0.01))}
    for t in (5.0, 6.0, 8.5):
        assert rows[round(t, 2)][1][6] == pytest.approx(rows[round(t - 0.3, 2)][0][6], abs=1e-12)


def test_noise_is_seeded_and_has_the_requested_spread():
    def samples(seed):
        s = AttackSchedule([make_attack(3, 'noise', (2,), 0.01, 0.0, 100.0, seed=seed)])
        return np.array([s.apply(t, Q, V, A)[0][2] - Q[2] for t in np.arange(0.0, 50.0, 0.01)])
    a, b, c = samples(1), samples(1), samples(2)
    np.testing.assert_array_equal(a, b)
    assert not np.array_equal(a, c)
    assert np.std(a) == pytest.approx(0.01, rel=0.05)
    assert abs(np.mean(a)) < 0.001


def test_drift_mimic_looks_like_growing_viscous_friction():
    kp = 600.0
    s = AttackSchedule([make_attack(1, 'drift_mimic', (3,), 0.002, 10.0, 20.0)])
    for t, fraction in ((10.0, 0.0), (20.0, 0.5), (29.99, 0.9995)):
        q2, v2, _, _ = s.apply(t, Q, V, A)
        equivalent_friction = -kp * (q2[3] - Q[3]) / V[3]
        assert equivalent_friction == pytest.approx(kp * 0.002 * fraction, rel=1e-6, abs=1e-9)


def test_inputs_are_not_modified():
    q_in, v_in, a_in = Q.copy(), V.copy(), A.copy()
    AttackSchedule([make_attack(1, 'scaling', (0, 1), 2.0, 0.0, 1.0)]).apply(0.5, q_in, v_in, a_in)
    np.testing.assert_array_equal(q_in, Q)
    np.testing.assert_array_equal(v_in, V)
    np.testing.assert_array_equal(a_in, A)


def test_events_are_selected_by_time():
    s = AttackSchedule([make_attack(2, 'ramp', (5,), 0.001, 30.0, 5.0),
                        make_attack(1, 'step', (0,), 0.02, 10.0, 5.0)])
    assert s.apply(12.0, Q, V)[3].event_id == 1
    assert s.apply(32.0, Q, V)[3].event_id == 2
    assert s.apply(20.0, Q, V)[3].event_id == 0


@pytest.mark.parametrize('kwargs', [
    {'kind': 'teleport'},
    {'joints': ()},
    {'joints': (7,)},
    {'joints': (2, 2)},
    {'duration': 0.0},
    {'start': -1.0},
    {'magnitude': float('inf')},
    {'kind': 'coordinated'},
    {'kind': 'sine', 'frequency': 0.0},
    {'kind': 'scaling', 'magnitude': -1.0},
    {'kind': 'noise', 'magnitude': -0.1},
    {'kind': 'replay', 'lag': 0.0},
    {'kind': 'delay', 'lag': -1.0},
    {'magnitudes': (0.1, 0.2)},
    {'event_id': 0},
])
def test_invalid_attacks_are_rejected(kwargs):
    args = dict(event_id=1, kind='step', joints=(3,), magnitude=0.02, start=1.0, duration=1.0)
    args.update(kwargs)
    with pytest.raises(ValueError):
        make_attack(**args)