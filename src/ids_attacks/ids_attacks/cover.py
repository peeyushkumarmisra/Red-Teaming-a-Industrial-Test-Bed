# =============================================================================
#  cover.py - falsified torque report (C24)
# -----------------------------------------------------------------------------
#  When the setpoint is falsified, the robot's controller produces a different torque,
#  and the IDS command check sees that torque in /robot/measurement. A careful
#  attacker also rewrites the reported torque. This class computes the change to
#  remove: the controller torque is Kp*(q_ref - q) + Kd*(v_ref - v), so a falsified
#  setpoint (dq, dv) adds Kp*(dq + dv*dt) + Kd*dv, where dt is the time since that
#  setpoint was received (held at most `hold` seconds).
#  Why it matters: with the torque evidence hidden, only the physics of the motion can
#  reveal the attack. v1.4 measured this once (docs/v1.4.md).
# =============================================================================

from collections import OrderedDict

import numpy as np

# Gains of the robot's reference-tracking controller (iiwa_sim/config/controllers.yaml); the
# attacker is assumed to know them.
IIWA_KP = (800.0, 800.0, 600.0, 600.0, 250.0, 150.0, 50.0)
IIWA_KD = (64.6, 81.6, 32.4, 32.0, 6.4, 3.8, 1.4)


class TorqueCover:

    def __init__(self, kp=IIWA_KP, kd=IIWA_KD, hold=0.05, history=1024):
        self._kp = np.array(kp, dtype=float)
        self._kd = np.array(kd, dtype=float)
        self._hold = float(hold)
        self._history = int(history)
        self._offsets = OrderedDict()

    # Store how far the setpoint with this sequence number was falsified.
    def remember(self, seq, stamp, dq, dv):
        dq = np.asarray(dq, dtype=float)
        dv = np.asarray(dv, dtype=float)
        if not np.any(dq) and not np.any(dv):
            self._offsets.pop(int(seq), None)
            return
        self._offsets[int(seq)] = (float(stamp), dq.copy(), dv.copy())
        while len(self._offsets) > self._history:
            self._offsets.popitem(last=False)

    # Extra torque the controller produced at time t because of that falsification; zero for
    # unknown or clean samples.
    def offset_torque(self, seq, t):
        entry = self._offsets.get(int(seq))
        if entry is None:
            return np.zeros_like(self._kp)
        stamp, dq, dv = entry
        dt = max(t - stamp, 0.0)
        if dt > self._hold:
            return self._kp * (dq + dv * self._hold)
        return self._kp * (dq + dv * dt) + self._kd * dv

    # Reported torque minus the extra torque: what the trusted reference would have produced.
    def falsify(self, effort, seq, t):
        return np.asarray(effort, dtype=float) - self.offset_torque(seq, t)
