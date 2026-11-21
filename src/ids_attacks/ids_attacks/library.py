from collections import deque
from dataclasses import dataclass
import math

import numpy as np

KINDS = ('step', 'ramp', 'sine', 'scaling', 'freeze', 'replay', 'delay', 'noise', 'coordinated',
         'drift_mimic')
LAGGED = ('replay', 'delay')
DOF = 7


@dataclass(frozen=True)
class Attack:
    event_id: int
    kind: str
    joints: tuple
    magnitude: float
    start: float
    duration: float
    frequency: float = 0.0
    lag: float = 0.0
    magnitudes: tuple = ()
    seed: int = 0

    @property
    def end(self):
        return self.start + self.duration

    def active(self, t):
        return self.start <= t < self.end

    def joint_magnitudes(self):
        if self.magnitudes:
            return tuple(self.magnitudes)
        return (self.magnitude,) * len(self.joints)


@dataclass(frozen=True)
class Truth:
    attack_active: bool
    kind: str
    joint_mask: tuple
    magnitude: float
    event_id: int


NO_ATTACK = Truth(False, '', (False,) * DOF, 0.0, 0)


def _finite(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{what} must be a finite number')
    return float(value)


def make_attack(event_id, kind, joints, magnitude=None, start=0.0, duration=0.0, frequency=0.0,
                lag=0.0, magnitudes=(), seed=0):
    if kind not in KINDS:
        raise ValueError(f'kind must be one of {KINDS}')
    if isinstance(event_id, bool) or not isinstance(event_id, int) or event_id < 1:
        raise ValueError('id must be a positive integer')
    joints = tuple(joints)
    if not joints or any(isinstance(j, bool) or not isinstance(j, int) or not 0 <= j < DOF
                         for j in joints):
        raise ValueError('joints must be joint numbers from 1 to 7')
    if len(set(joints)) != len(joints):
        raise ValueError('joints must not repeat')
    start = _finite(start, 'start')
    duration = _finite(duration, 'duration')
    if start < 0.0 or duration <= 0.0:
        raise ValueError('start must be >= 0 and duration > 0')
    magnitudes = tuple(_finite(m, 'magnitudes') for m in (magnitudes or ()))
    if magnitudes and len(magnitudes) != len(joints):
        raise ValueError('magnitudes must have one value per joint')
    if magnitude is None:
        magnitude = max((abs(m) for m in magnitudes), default=0.0)
    magnitude = _finite(magnitude, 'magnitude')
    frequency = _finite(frequency, 'frequency')
    lag = _finite(lag, 'lag')
    values = magnitudes or (magnitude,)
    if kind == 'coordinated' and len(joints) < 2:
        raise ValueError('coordinated attacks need at least two joints')
    if kind == 'sine' and frequency <= 0.0:
        raise ValueError('sine attacks need frequency > 0')
    if kind == 'scaling' and any(m <= 0.0 for m in values):
        raise ValueError('scaling factors must be positive')
    if kind == 'noise' and any(m < 0.0 for m in values):
        raise ValueError('noise magnitudes must be >= 0')
    if kind in LAGGED and lag <= 0.0:
        raise ValueError(f'{kind} attacks need lag > 0')
    if kind == 'drift_mimic' and any(m < 0.0 for m in values):
        raise ValueError('drift_mimic magnitudes must be >= 0')
    return Attack(event_id, kind, joints, magnitude, start, duration, frequency, lag, magnitudes,
                  int(seed))


class AttackSchedule:

    def __init__(self, attacks=()):
        self._attacks = sorted(attacks, key=lambda a: a.start)
        lags = [a.lag for a in self._attacks if a.kind in LAGGED]
        self._keep = (max(lags) + 1.0) if lags else 1.0
        self._history = deque()
        self._state = {}

    @property
    def attacks(self):
        return list(self._attacks)

    def current(self, t):
        for attack in self._attacks:
            if attack.active(t):
                return attack
        return None

    def apply(self, t, position, velocity, acceleration=None):
        q = np.array(position, dtype=float)
        v = np.array(velocity, dtype=float)
        a = np.zeros(DOF) if acceleration is None else np.array(acceleration, dtype=float)
        self._history.append((t, q.copy(), v.copy(), a.copy()))
        while len(self._history) > 1 and self._history[0][0] < t - self._keep:
            self._history.popleft()
        attack = self.current(t)
        if attack is None:
            return q, v, a, NO_ATTACK
        state = self._state.get(attack.event_id)
        if state is None:
            state = self._start(attack, q, v, a)
            self._state[attack.event_id] = state
        q2, v2, a2 = q.copy(), v.copy(), a.copy()
        tau = t - attack.start
        for j, m in zip(attack.joints, attack.joint_magnitudes()):
            self._effect(attack, state, j, m, t, tau, q, v, a, q2, v2, a2)
        mask = tuple(i in attack.joints for i in range(DOF))
        return q2, v2, a2, Truth(True, attack.kind, mask, attack.magnitude, attack.event_id)

    def _start(self, attack, q, v, a):
        state = {'q0': q.copy(), 'v0': v.copy(), 'a0': a.copy()}
        if attack.kind == 'replay':
            t0 = attack.start - attack.lag
            segment = [s for s in self._history if t0 <= s[0] < attack.start]
            if not segment:
                segment = [self._history[0]]
            state['segment'] = segment
        if attack.kind == 'noise':
            state['rng'] = np.random.default_rng([attack.seed, attack.event_id])
        return state

    def _lagged(self, t):
        target = None
        for sample in self._history:
            if sample[0] <= t:
                target = sample
            else:
                break
        return target if target is not None else self._history[0]

    def _effect(self, attack, state, j, m, t, tau, q, v, a, q2, v2, a2):
        kind = attack.kind
        if kind in ('step', 'coordinated'):
            q2[j] += m
        elif kind == 'ramp':
            q2[j] += m * tau
            v2[j] += m
        elif kind == 'sine':
            w = 2.0 * math.pi * attack.frequency
            q2[j] += m * math.sin(w * tau)
            v2[j] += m * w * math.cos(w * tau)
            a2[j] -= m * w * w * math.sin(w * tau)
        elif kind == 'scaling':
            q2[j] = state['q0'][j] + m * (q[j] - state['q0'][j])
            v2[j] = m * v[j]
            a2[j] = m * a[j]
        elif kind == 'freeze':
            q2[j] = state['q0'][j]
            v2[j] = 0.0
            a2[j] = 0.0
        elif kind == 'replay':
            segment = state['segment']
            period = max(attack.lag, 1e-9)
            target = segment[0][0] + math.fmod(tau, period)
            chosen = segment[0]
            for sample in segment:
                if sample[0] <= target:
                    chosen = sample
                else:
                    break
            q2[j], v2[j], a2[j] = chosen[1][j], chosen[2][j], chosen[3][j]
        elif kind == 'delay':
            sample = self._lagged(t - attack.lag)
            q2[j], v2[j], a2[j] = sample[1][j], sample[2][j], sample[3][j]
        elif kind == 'noise':
            q2[j] += state['rng'].normal(0.0, m) if m > 0.0 else 0.0
        elif kind == 'drift_mimic':
            c = m * min(tau / attack.duration, 1.0)
            rate = m / attack.duration if tau < attack.duration else 0.0
            q2[j] -= c * v[j]
            v2[j] -= c * a[j] + rate * v[j]