# =============================================================================
#  test_baselines.py - baseline statistics, limits and the momentum observer
# -----------------------------------------------------------------------------
#  The observer is checked on a simulated two-link arm: no residual without an external torque,
#  the torque itself with one.
# =============================================================================

import math

import numpy as np
import pytest

from ids_evaluation import baselines, layout

URDF = """<?xml version="1.0"?>
<robot name="two_link">
  <link name="base"/>
  <link name="l1">
    <inertial><origin xyz="0 0 0.25"/><mass value="2.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="l2">
    <inertial><origin xyz="0 0 0.2"/><mass value="1.0"/>
      <inertia ixx="0.02" iyy="0.02" izz="0.005" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 1 0"/><limit effort="100" velocity="2" lower="-3" upper="3"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit effort="100" velocity="2" lower="-3" upper="3"/></joint>
</robot>
"""


def test_ewma_statistic_is_unit_scale_on_white_noise():
    z = np.random.default_rng(1).normal(size=(200000, 2))
    s = baselines.ewma_statistic(z, 0.2)
    assert s.shape == z.shape
    assert math.isclose(np.sqrt(np.mean(s[1000:] ** 2)), 1.0, rel_tol=0.02)


def test_standardiser():
    x = np.random.default_rng(2).normal(3.0, 2.0, size=(10000, 3))
    scale = baselines.Standardiser.fit(x)
    z = scale(x)
    assert np.allclose(z.mean(axis=0), 0.0, atol=1e-9)
    assert np.allclose(z.std(axis=0), 1.0, atol=1e-9)
    flat = baselines.Standardiser.fit(np.ones((5, 2)))
    assert np.all(flat.std > 0)


def test_lowest_quiet_limit_is_quiet_on_training():
    s = np.abs(np.random.default_rng(3).normal(size=(5000, 3)))
    limit = baselines.lowest_quiet_limit(s)
    alarm, _ = baselines.alarm_series(s, limit)
    assert not alarm.any()
    alarm, _ = baselines.alarm_series(s, limit * 0.9)
    assert alarm.any()
    with pytest.raises(ValueError):
        baselines.lowest_quiet_limit(s[:3])


def test_alarm_series_debounce_hysteresis_and_primary():
    s = np.zeros((40, 3))
    s[5:9, 1] = 2.0
    s[15:25, 2] = 2.0
    s[25:28, 2] = 0.8
    alarm, primary = baselines.alarm_series(s, 1.0)
    assert not alarm[5:15].any()
    assert not alarm[15:19].any() and alarm[19]
    assert alarm[19:28].all()
    first_clear = 28 + 4
    assert alarm[first_clear - 1] and not alarm[first_clear]
    assert (primary[alarm] == 2).all() and (primary[~alarm] == -1).all()


def test_status_from_alarms():
    s = np.ones((3, 7))
    st = baselines.status_from_alarms(np.array([0, 1, 1]), np.array([-1, 4, 4]), s, 2.0)
    assert st.shape == (3, layout.STATUS_COLUMNS)
    assert (st[:, layout.MODE] == layout.ARMED).all()
    assert list(st[:, layout.ALARM]) == [0, 1, 1]
    assert (st[:, layout.LIMIT] == 2.0).all()


def test_context_mass_follows_flag():
    ref = np.zeros((3, layout.REFERENCE_COLUMNS))
    ref[:, 0] = [0.0, 1.0, 2.0]
    ref[:, 2] = [0, 1, 2]
    ref[:, 3] = [5.0, 5.0, 3.0]
    m = baselines.context_mass(ref, np.array([-1.0, 0.5, 1.5, 2.5]))
    assert list(m) == [0.0, 0.0, 5.0, 3.0]


def test_measurement_arrays():
    m = np.arange(2 * layout.MEASUREMENT_COLUMNS, dtype=float).reshape(2, -1)
    t, q, v, tau = baselines.measurement_arrays(m)
    assert t[0] == 0 and q[0, 0] == 3 and v[0, 0] == 10 and tau[0, 6] == 23


def _simulate(external, duration=4.0, sample=0.01, substeps=50):
    pin = pytest.importorskip('pinocchio')
    model = pin.buildModelFromXML(URDF)
    data = model.createData()
    q = np.array([0.3, -0.4])
    v = np.zeros(2)
    h = sample / substeps
    steps = int(round(duration / sample))
    out = np.zeros((steps, 3, 2))
    for k in range(steps):
        tt = k * sample
        target = np.array([0.3 + 0.4 * np.sin(tt), -0.4 + 0.3 * np.sin(1.7 * tt)])
        tau = (pin.computeGeneralizedGravity(model, data, q) + 10.0 * (target - q)
               - 1.0 * v)
        out[k] = q, v, tau
        for _ in range(substeps):
            a = pin.aba(model, data, q, v, tau + external(tt))
            v = v + a * h
            q = q + v * h
    return np.arange(steps) * sample, out


def test_momentum_observer_tracks_external_torque():
    pytest.importorskip('pinocchio')
    ext = np.array([0.0, 1.5])
    t, sim = _simulate(lambda tt: ext if tt >= 1.5 else np.zeros(2))
    obs = baselines.MomentumObserver(URDF, gain=5.0, tip_joint='j2')
    r = obs.residual(t, sim[:, 0], sim[:, 1], sim[:, 2], np.zeros(len(t)))
    assert np.abs(r[(t > 0.5) & (t < 1.5)]).max() < 0.1
    assert np.allclose(r[t > 3.0].mean(axis=0), ext, atol=0.1)


def test_momentum_observer_resets_after_payload_change():
    pytest.importorskip('pinocchio')
    t, sim = _simulate(lambda tt: np.zeros(2), duration=1.0)
    mass = np.where(t >= 0.5, 1.0, 0.0)
    obs = baselines.MomentumObserver(URDF, gain=50.0, tip_joint='j2', guard=0.1)
    r = obs.residual(t, sim[:, 0], sim[:, 1], sim[:, 2], mass)
    assert np.all(r[(t >= 0.5) & (t < 0.6)] == 0.0)
