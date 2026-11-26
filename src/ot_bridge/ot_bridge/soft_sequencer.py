import numpy as np

from ids_experiments.sequencer import Command, Dwell, Move, SetContext
from ot_bridge.modbus_map import CellCommand, KIND_DWELL, KIND_IDLE, KIND_MOVE

ATTACH_NAMES = ('attach',)
DETACH_NAMES = ('detach', 'release')


class SoftSequencer:

    def __init__(self, start, program, passes=1):
        self._hold = np.asarray(start, dtype=float).copy()
        self._from = self._hold.copy()
        self._program = list(program)
        self._passes = passes
        self._completed = 0
        self._index = 0
        self._started_at = None
        self._context = 0
        self._gripper = 0
        self._seq = 0
        self._finished = not self._program

    @property
    def completed_passes(self):
        return self._completed

    @property
    def finished(self):
        return self._finished

    def scan(self, t):
        index = self._index
        kind = KIND_IDLE
        target = self._hold.copy()
        duration_ms = 0
        seat = None
        if not self._finished:
            step = self._program[self._index]
            if self._started_at is None:
                self._started_at = t
                self._from = self._hold.copy()
                if isinstance(step, Command):
                    if step.name in ATTACH_NAMES:
                        self._gripper = 1
                    elif step.name in DETACH_NAMES:
                        self._gripper = 0
                    elif step.name == 'seat':
                        seat = step.args
                elif isinstance(step, SetContext):
                    self._context = int(step.context)
            elapsed = t - self._started_at
            done = True
            if isinstance(step, Move):
                kind = KIND_MOVE
                target = np.asarray(step.target, dtype=float)
                duration_ms = int(round(step.duration * 1000.0))
                done = elapsed >= step.duration - 1e-9
                if done:
                    self._hold = target.copy()
            elif isinstance(step, Dwell):
                kind = KIND_DWELL
                target = self._hold.copy()
                done = elapsed >= step.duration - 1e-9
            if done:
                self._advance()
        command = CellCommand(seq=self._seq, step=index, kind=kind, duration_ms=duration_ms,
                              context=self._context, gripper=self._gripper,
                              passes=self._completed, finished=int(self._finished),
                              q_target=target)
        self._seq += 1
        return command, seat

    def _advance(self):
        self._started_at = None
        self._index += 1
        if self._index >= len(self._program):
            self._index = 0
            self._completed += 1
            if self._passes and self._completed >= self._passes:
                self._finished = True
                