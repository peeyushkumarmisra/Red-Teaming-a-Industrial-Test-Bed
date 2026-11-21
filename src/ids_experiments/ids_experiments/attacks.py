# =============================================================================
#  attacks.py - setpoint attacks the MitM can apply
# -----------------------------------------------------------------------------
#  step: add a constant offset (rad) to one joint's position setpoint.
#  ramp: add an offset that grows at a constant rate (rad/s), the slow, stealthy case.
#  The schedule also produces the ground truth used to score the IDS.
# =============================================================================

from dataclasses import dataclass

import numpy as np

KINDS = ('step', 'ramp')
DOF = 7


@dataclass(frozen=True)
# One attack event; joint is 0-based here (0 = a1).
class Attack:
    event_id: int
    kind: str
    joint: int
    magnitude: float
    start: float
    duration: float

    @property
    def end(self):
        return self.start + self.duration

    def active(self, t):
        return self.start <= t < self.end

    # Change to (position, velocity) at time t of the attack.
    def offset(self, t):
        if self.kind == 'step':
            return self.magnitude, 0.0
        return self.magnitude * (t - self.start), self.magnitude


@dataclass(frozen=True)
# Ground truth for one forwarded sample.
class Truth:
    attack_active: bool
    kind: str
    joint_mask: tuple
    magnitude: float
    event_id: int


# All attacks of a scenario; at most one is active at a time.
class AttackSchedule:

    def __init__(self, attacks=()):
        self._attacks = sorted(attacks, key=lambda a: a.start)

    @property
    def attacks(self):
        return list(self._attacks)

    def current(self, t):
        for attack in self._attacks:
            if attack.active(t):
                return attack
        return None

    # Return the (possibly falsified) setpoint and the truth. Inputs are not modified.
    def apply(self, t, position, velocity):
        position = np.array(position, dtype=float)
        velocity = np.array(velocity, dtype=float)
        attack = self.current(t)
        if attack is None:
            return position, velocity, Truth(False, '', (False,) * DOF, 0.0, 0)
        dq, dv = attack.offset(t)
        position[attack.joint] += dq
        velocity[attack.joint] += dv
        mask = tuple(i == attack.joint for i in range(DOF))
        return position, velocity, Truth(True, attack.kind, mask, attack.magnitude,
                                         attack.event_id)
