import numpy as np

from ot_bridge.modbus_map import (CellCommand, COMMAND_REGISTERS, DOF, KIND_MOVE, Q_SCALE,
                                  REG_QTARGET, decode_angle, encode_angle, poison_registers)


def test_angle_round_trip_within_one_lsb():
    for q in np.linspace(-2.9, 2.9, 59):
        assert abs(decode_angle(encode_angle(q)) - q) <= 1.0 / Q_SCALE


def test_command_round_trip_through_registers():
    command = CellCommand(seq=7, step=3, kind=KIND_MOVE, duration_ms=4000, context=1,
                          gripper=1, q_target=np.array([0.1, -0.2, 0.3, -1.2, 0.0, 0.8, 0.05]))
    regs = command.to_registers()
    assert len(regs) == COMMAND_REGISTERS
    back = CellCommand.from_registers(regs)
    assert back.step == 3 and back.kind == KIND_MOVE and back.context == 1
    assert back.duration_ms == 4000 and back.gripper == 1 and back.seq == 7
    assert np.abs(back.q_target - command.q_target).max() <= 1.0 / Q_SCALE


def test_poison_adds_a_fixed_offset_to_one_joint():
    regs = CellCommand(q_target=np.zeros(DOF)).to_registers()
    poisoned = poison_registers(regs, 3, 0.02)
    back = CellCommand.from_registers(poisoned)
    assert abs(back.q_target[3] - 0.02) <= 1.0 / Q_SCALE
    assert np.abs(np.delete(back.q_target, 3)).max() <= 1.0 / Q_SCALE
    assert poisoned[REG_QTARGET + 3] != regs[REG_QTARGET + 3]


def test_registers_stay_inside_unsigned_16_bit():
    regs = CellCommand(seq=70000, step=70000,
                       q_target=np.array([5.0, -5.0, 0, 0, 0, 0, 0])).to_registers()
    assert all(0 <= r <= 0xFFFF for r in regs)