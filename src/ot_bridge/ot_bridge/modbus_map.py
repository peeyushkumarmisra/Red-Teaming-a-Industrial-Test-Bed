from dataclasses import dataclass, field
from typing import List

import numpy as np

DOF = 7
Q_SCALE = 10_000.0
Q_OFFSET = 3.3

KIND_IDLE = 0
KIND_MOVE = 1
KIND_DWELL = 2

REG_SEQ = 0
REG_STEP = 1
REG_KIND = 2
REG_DURATION_MS = 3
REG_CONTEXT = 4
REG_GRIPPER = 5
REG_PASSES = 6
REG_FINISHED = 7
REG_QTARGET = 10
COMMAND_REGISTERS = REG_QTARGET + DOF

COIL_RUN = 0
COIL_IDS_ALARM = 8
COIL_SAFE_STOP = 9
COIL_COUNT = 16

STATUS_BASE = 40
STATUS_HALTED = 0
STATUS_STEP_ACK = 1
STATUS_REFERENCE_SEQ = 2
STATUS_REGISTERS = 4

MOVE_DURATION_MAX_MS = 65_000


def _clip_u16(value):
    return int(value) & 0xFFFF


def encode_angle(q):
    return _clip_u16(max(0, min(0xFFFF, int(round((float(q) + Q_OFFSET) * Q_SCALE)))))


def decode_angle(reg):
    return (int(reg) & 0xFFFF) / Q_SCALE - Q_OFFSET


@dataclass
class CellCommand:
    seq: int = 0
    step: int = 0
    kind: int = KIND_IDLE
    duration_ms: int = 0
    context: int = 0
    gripper: int = 0
    passes: int = 0
    finished: int = 0
    q_target: np.ndarray = field(default_factory=lambda: np.zeros(DOF))

    def to_registers(self):
        regs = [0] * COMMAND_REGISTERS
        regs[REG_SEQ] = _clip_u16(self.seq)
        regs[REG_STEP] = _clip_u16(self.step)
        regs[REG_KIND] = _clip_u16(self.kind)
        regs[REG_DURATION_MS] = _clip_u16(min(self.duration_ms, MOVE_DURATION_MAX_MS))
        regs[REG_CONTEXT] = _clip_u16(self.context)
        regs[REG_GRIPPER] = _clip_u16(self.gripper)
        regs[REG_PASSES] = _clip_u16(self.passes)
        regs[REG_FINISHED] = _clip_u16(self.finished)
        target = np.asarray(self.q_target, dtype=float)
        for j in range(DOF):
            regs[REG_QTARGET + j] = encode_angle(target[j])
        return regs

    @classmethod
    def from_registers(cls, regs):
        target = np.array([decode_angle(regs[REG_QTARGET + j]) for j in range(DOF)])
        return cls(seq=regs[REG_SEQ], step=regs[REG_STEP], kind=regs[REG_KIND],
                   duration_ms=regs[REG_DURATION_MS], context=regs[REG_CONTEXT],
                   gripper=regs[REG_GRIPPER], passes=regs[REG_PASSES],
                   finished=regs[REG_FINISHED], q_target=target)


def poison_registers(regs: List[int], joint_index: int, delta_rad: float):
    out = list(regs)
    out[REG_QTARGET + joint_index] = _clip_u16(
        max(0, min(0xFFFF, out[REG_QTARGET + joint_index] + int(round(delta_rad * Q_SCALE)))))
    return out
