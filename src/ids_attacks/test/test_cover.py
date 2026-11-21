# =============================================================================
#  test_cover.py - the cover hides the attack's torque exactly
# -----------------------------------------------------------------------------
#  A model of the controller computes the torque for the falsified and the trusted
#  setpoint; after the cover the report must equal the trusted one for step, ramp and
#  sine at several delays. Clean samples pass unchanged; the history stays bounded.
# =============================================================================

import numpy as np
import pytest

from ids_attacks.cover import IIWA_KD, IIWA_KP, TorqueCover
from ids_attacks.library import AttackSchedule, make_attack

KP = np.array(IIWA_KP)
KD = np.array(IIWA_KD)
HOLD = 0.05


def controller_torque(sample_q, sample_v, stamp, t, q, v):
    dt = t - stamp
    if dt > HOLD:
        ref_q, ref_v = sample_q + sample_v * HOLD, np.zeros(7)
    else:
        ref_q, ref_v = sample_q + sample_v * max(dt, 0.0), sample_v
    return KP * (ref_q - q) + KD * (ref_v - v)


@pytest.mark.parametrize('kind,magnitude', [('step', 0.02), ('ramp', 0.001), ('sine', 0.01)])
@pytest.mark.parametrize('dt', [0.0, 0.004, 0.009, 0.08])
def test_cover_reports_the_torque_of_the_trusted_reference(kind, magnitude, dt):
    schedule = AttackSchedule([make_attack(1, kind, (3,), magnitude, 0.0, 10.0, frequency=0.5)])
    cover = TorqueCover()
    stamp = 2.0
    trusted_q = np.linspace(-0.5, 0.5, 7)
    trusted_v = np.full(7, 0.2)
    q2, v2, _, _ = schedule.apply(stamp, trusted_q, trusted_v)
    cover.remember(42, stamp, q2 - trusted_q, v2 - trusted_v)
    q_meas = trusted_q + 0.003
    v_meas = trusted_v - 0.01
    applied = controller_torque(q2, v2, stamp, stamp + dt, q_meas, v_meas)
    expected = controller_torque(trusted_q, trusted_v, stamp, stamp + dt, q_meas, v_meas)
    np.testing.assert_allclose(cover.falsify(applied, 42, stamp + dt), expected, atol=1e-9)


def test_unknown_or_clean_samples_are_reported_unchanged():
    cover = TorqueCover()
    effort = np.arange(7.0)
    np.testing.assert_array_equal(cover.falsify(effort, 5, 1.0), effort)
    cover.remember(6, 1.0, np.zeros(7), np.zeros(7))
    np.testing.assert_array_equal(cover.falsify(effort, 6, 1.0), effort)


def test_history_is_bounded():
    cover = TorqueCover(history=3)
    for seq in range(10):
        cover.remember(seq, float(seq), np.ones(7), np.zeros(7))
    assert not np.any(cover.offset_torque(0, 0.0))
    assert np.all(cover.offset_torque(9, 9.0) > 0.0)
