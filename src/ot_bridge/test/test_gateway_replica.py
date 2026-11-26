import numpy as np

from ids_experiments.sequencer import shuttle_program
from ot_bridge.ids_replica import ReplicaCore
from ot_bridge.modbus_io import LocalStore
from ot_bridge.modbus_map import (COIL_SAFE_STOP, COMMAND_REGISTERS,
                                  poison_registers)
from ot_bridge.robot_gateway import GatewayCore
from ot_bridge.soft_plc import PlcCore

START = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])


def _program():
    def station(seed):
        r = np.random.default_rng(seed)
        base = START + r.uniform(-0.4, 0.4, 7)
        return {'hover': base, 'grab': base + r.uniform(-0.1, 0.1, 7),
                'drop': base + r.uniform(-0.1, 0.1, 7)}
    return shuttle_program(station(1), station(2), 5.0, 4.0, 2.0, 0.5, ((0.0, 0.5), (-0.5, 0.0)))


def test_gateway_and_replica_agree_without_an_attack():
    truth, control = LocalStore(), LocalStore()
    plc = PlcCore(START, _program(), passes=1)
    gateway, replica = GatewayCore(START), ReplicaCore(START)
    worst = 0.0
    for k in range(5500):
        t = k * 0.01
        plc.scan(truth)
        control.write_holding(0, truth.read_holding(0, COMMAND_REGISTERS))
        control.write_coil(COIL_SAFE_STOP, truth.read_coil(COIL_SAFE_STOP))
        g = gateway.tick(t, control)
        r = replica.tick(t, truth)
        worst = max(worst, float(np.abs(g['position'] - r['position']).max()))
    assert worst == 0.0


def test_setpoint_mitm_diverges_only_on_the_attacked_joint():
    truth, control = LocalStore(), LocalStore()
    plc = PlcCore(START, _program(), passes=1)
    gateway, replica = GatewayCore(START), ReplicaCore(START)
    joint, gamma = 3, 0.02
    diffs = []
    for k in range(5500):
        t = k * 0.01
        plc.scan(truth)
        registers = truth.read_holding(0, COMMAND_REGISTERS)
        if 1500 <= k < 3000:
            registers = poison_registers(registers, joint, gamma)
        control.write_holding(0, registers)
        g = gateway.tick(t, control)
        r = replica.tick(t, truth)
        if 1500 <= k < 3000:
            diffs.append(g['position'] - r['position'])
    diffs = np.array(diffs)
    assert abs(np.abs(diffs[:, joint]).max() - gamma) < 1e-4
    assert np.abs(np.delete(diffs, joint, axis=1)).max() < 1e-9


def test_gateway_reports_safe_stop_from_the_coil():
    store = LocalStore()
    PlcCore(START, _program(), passes=1).scan(store)
    store.write_coil(COIL_SAFE_STOP, True)
    out = GatewayCore(START).tick(0.0, store)
    assert out['safe_stop']