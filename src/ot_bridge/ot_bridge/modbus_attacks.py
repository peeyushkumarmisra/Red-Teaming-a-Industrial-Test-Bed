from dataclasses import dataclass

import numpy as np

from ids_attacks.library import AttackSchedule, DOF, Truth
from ot_bridge.modbus_map import CellCommand, REG_CONTEXT


@dataclass(frozen=True)
class PayloadPoison:
    event_id: int
    start: float
    duration: float
    value: int

    def active(self, t):
        return self.start <= t < self.start + self.duration


@dataclass(frozen=True)
class BlockAttack:
    event_id: int
    kind: str
    start: float
    duration: float
    period: float = 1.0

    def active(self, t):
        return self.start <= t < self.start + self.duration


def _payload_truth(event_id):
    return Truth(True, 'payload_poison', (False,) * DOF, 0.0, event_id)


def _block_truth(attack):
    return Truth(True, f'modbus_{attack.kind}', (False,) * DOF, 0.0, attack.event_id)


class ModbusAttacker:

    def __init__(self, schedule=None, payload=None, block=None):
        self._schedule = schedule if schedule is not None else AttackSchedule()
        self._payload = payload
        self._block = block
        self._frozen = None
        self._record = []
        self._record_window = (block.start - block.period, block.start) if block else None

    def transform(self, t, registers):
        registers = list(registers)
        if self._block is not None and self._block.kind == 'replay':
            if self._record_window[0] <= t < self._block.start:
                self._record.append((t, list(registers)))
        if self._block is not None and self._block.active(t):
            return self._apply_block(t, registers)
        self._frozen = None
        command = CellCommand.from_registers(registers)
        q2, _v, _a, truth = self._schedule.apply(t, command.q_target, np.zeros(DOF), np.zeros(DOF))
        command.q_target = np.asarray(q2, dtype=float)
        out = command.to_registers()
        if self._payload is not None and self._payload.active(t):
            out[REG_CONTEXT] = int(self._payload.value) & 0xFFFF
            if not truth.attack_active:
                truth = _payload_truth(self._payload.event_id)
        return out, truth

    def _apply_block(self, t, registers):
        if self._block.kind == 'freeze':
            if self._frozen is None:
                self._frozen = list(registers)
            return list(self._frozen), _block_truth(self._block)
        if self._block.kind == 'replay' and self._record:
            tau = t - self._block.start
            period = max(self._block.period, 1e-9)
            base = self._record[0][0]
            target = base + (tau % period)
            chosen = self._record[0][1]
            for stamp, regs in self._record:
                if stamp <= target:
                    chosen = regs
                else:
                    break
            return list(chosen), _block_truth(self._block)
        return registers, _block_truth(self._block)


def attacker_from_scenario(scenario):
    schedule = AttackSchedule(list(getattr(scenario, 'attacks', ()) or ()))
    payload = None
    block = None
    for i, spec in enumerate(getattr(scenario, 'modbus_attacks', ()) or (), start=1):
        kind = spec['kind']
        if kind == 'payload_poison':
            payload = PayloadPoison(spec.get('id', i), spec['start'], spec['duration'],
                                    int(spec.get('value', 0)))
        elif kind in ('modbus_freeze', 'modbus_replay'):
            block = BlockAttack(spec.get('id', i), kind.split('_', 1)[1], spec['start'],
                                spec['duration'], spec.get('period', 1.0))
    return ModbusAttacker(schedule=schedule, payload=payload, block=block)
