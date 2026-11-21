# =============================================================================
#  baselines.py - simpler detectors to compare the IDS with (C22)
# -----------------------------------------------------------------------------
#  static:   |z| > limit on the standardised disturbance estimate of the v2 IDS.
#  ewma:     one EWMA (lambda 0.2) of z instead of the fast/slow pair.
#  momentum: a generalised-momentum observer on the measured torque, the classical
#            collision-detection residual: r = K*(p - p0 - integral(tau + C^T v - g + r)).
#  Every baseline gets the same debounce and hysteresis as the IDS and the lowest limit
#  that raises no alarm on the training part of a nominal run.
#  Why it matters: shows what the closed-loop model, the two time scales and the limit
#  selection add over simpler designs.
# =============================================================================

from dataclasses import dataclass
import math

import numpy as np

from ids_evaluation import layout


@dataclass(frozen=True)
# Debounce 5 windows to raise, 5 windows below half the limit to clear: the IDS rules.
class DetectorRules:
    debounce: int = 5
    clear_count: int = 5
    hysteresis: float = 0.5


@dataclass(frozen=True)
class Standardiser:
    mean: np.ndarray
    std: np.ndarray

    @classmethod
    def fit(cls, x, floor=1e-9):
        x = np.asarray(x, dtype=float)
        return cls(x.mean(axis=0), np.maximum(x.std(axis=0), floor))

    def __call__(self, x):
        return (np.asarray(x, dtype=float) - self.mean) / self.std


def static_statistic(z):
    return np.abs(z)


def ewma_statistic(z, lam=0.2):
    z = np.asarray(z, dtype=float)
    out = np.empty_like(z)
    e = np.zeros(z.shape[1])
    for i, row in enumerate(z):
        e = lam * row + (1.0 - lam) * e
        out[i] = e
    return np.abs(out) / math.sqrt(lam / (2.0 - lam))


# Smallest limit with no debounced alarm on the training data: a favourable choice for the
# baseline.
def lowest_quiet_limit(statistic, rules=DetectorRules(), skip=0):
    s = np.asarray(statistic, dtype=float)[skip:]
    n = rules.debounce
    if len(s) < n:
        raise ValueError('training data shorter than the debounce')
    windows = np.lib.stride_tricks.sliding_window_view(s, n, axis=0)
    return float(windows.min(axis=-1).max()) * (1.0 + 1e-9) + 1e-12


# Per-joint debounce and hysteresis; the primary joint is the one furthest above its limit at
# onset.
def alarm_series(statistic, limit, rules=DetectorRules()):
    s = np.asarray(statistic, dtype=float)
    n, k = s.shape
    lim = np.broadcast_to(np.asarray(limit, dtype=float), (k,))
    over = np.zeros(k, dtype=int)
    under = np.zeros(k, dtype=int)
    active = np.zeros(k, dtype=bool)
    alarm = np.zeros(n, dtype=bool)
    primary = np.full(n, -1)
    current = -1
    for i in range(n):
        row = s[i]
        above = row > lim
        below = row < rules.hysteresis * lim
        over = np.where(~active & above, over + 1, np.where(active, over, 0))
        under = np.where(active & below, under + 1, np.where(active, 0, under))
        start = ~active & (over >= rules.debounce)
        stop = active & (under >= rules.clear_count)
        active = (active | start) & ~stop
        over[stop] = 0
        under[start] = 0
        alarm[i] = active.any()
        if alarm[i] and (i == 0 or not alarm[i - 1]):
            current = int(np.argmax(row / lim))
        primary[i] = current if alarm[i] else -1
    return alarm, primary


def status_from_alarms(alarm, primary, statistic, limit):
    n = len(alarm)
    status = np.zeros((n, layout.STATUS_COLUMNS))
    status[:, layout.MODE] = layout.ARMED
    status[:, layout.ALARM] = alarm
    status[:, layout.PRIMARY] = primary
    status[:, layout.FAST] = statistic
    status[:, layout.LIMIT] = limit
    return status


# Payload mass in effect at each time: the PLC's payload context and mass, as the controller
# and the IDS use them.
def context_mass(reference, t):
    ref = np.asarray(reference, dtype=float)
    idx = np.searchsorted(ref[:, 0], t, side='right') - 1
    out = np.zeros(len(t))
    ok = idx >= 0
    out[ok] = np.where(ref[idx[ok], 2] != 0.0, ref[idx[ok], 3], 0.0)
    return out


# Uses Pinocchio with the robot URDF and the nominal payload at the tool.
class MomentumObserver:

    def __init__(self, urdf, gain=5.0, tip_joint='joint_a7', payload_com=(0.0, 0.0, 0.204),
                 payload_inertia=0.008333, guard=0.3):
        import pinocchio as pin
        self._pin = pin
        self.model = pin.buildModelFromXML(urdf)
        self.data = self.model.createData()
        self.tip = self.model.getJointId(tip_joint)
        self.base = self.model.inertias[self.tip].copy()
        self.gain = float(gain)
        self.com = np.array(payload_com, dtype=float)
        self.inertia = np.eye(3) * float(payload_inertia)
        self.guard = float(guard)
        self._mass = None

    def _payload(self, mass):
        if mass == self._mass:
            return
        pin = self._pin
        self.model.inertias[self.tip] = (self.base + pin.Inertia(mass, self.com, self.inertia)
                                         if mass > 0.0 else self.base.copy())
        self.data = self.model.createData()
        self._mass = mass

    def residual(self, t, q, v, tau, mass):
        pin = self._pin
        n = len(t)
        out = np.zeros((n, q.shape[1]))
        r = np.zeros(q.shape[1])
        p0 = None
        f_prev = None
        integral = np.zeros(q.shape[1])
        changed = -math.inf
        for k in range(n):
            if self._mass is not None and mass[k] != self._mass:
                changed = t[k]
            self._payload(float(mass[k]))
            m = pin.crba(self.model, self.data, q[k])
            m = np.triu(m) + np.triu(m, 1).T
            c = pin.computeCoriolisMatrix(self.model, self.data, q[k], v[k])
            g = pin.computeGeneralizedGravity(self.model, self.data, q[k])
            p = m @ v[k]
            f = tau[k] + c.T @ v[k] - g
            # Restart after a payload change, because the model jumps.
            if p0 is None or t[k] - changed < self.guard:
                p0, integral, r = p, np.zeros_like(p), np.zeros_like(p)
            else:
                dt = t[k] - t[k - 1]
                # Trapezoid integration: accurate at the 100 Hz measurement rate.
                integral = integral + (0.5 * (f + f_prev) + r) * dt
                r = self.gain * (p - p0 - integral)
            f_prev = f
            out[k] = r
        return out


def measurement_arrays(measurement):
    m = np.asarray(measurement, dtype=float)
    return m[:, 0], m[:, 3:10], m[:, 10:17], m[:, 17:24]
