import numpy as np

from ids_experiments.trajectory import quintic
from ot_bridge.modbus_map import DOF, KIND_MOVE


class TrajectoryGenerator:

    def __init__(self, start):
        self._hold = np.asarray(start, dtype=float).copy()
        self._from = self._hold.copy()
        self._step = None
        self._started_at = None

    @property
    def hold(self):
        return self._hold.copy()

    def step(self, t, command):
        target = np.asarray(command.q_target, dtype=float)
        if command.step != self._step:
            self._step = command.step
            self._started_at = t
            self._from = self._hold.copy()
        elapsed = t - self._started_at
        if command.kind == KIND_MOVE:
            duration = command.duration_ms / 1000.0
            position, velocity, acceleration = quintic(self._from, target, duration, elapsed)
            if duration <= 0.0 or elapsed >= duration - 1e-9:
                self._hold = target.copy()
            return position, velocity, acceleration
        self._hold = target.copy()
        return target.copy(), np.zeros(DOF), np.zeros(DOF)