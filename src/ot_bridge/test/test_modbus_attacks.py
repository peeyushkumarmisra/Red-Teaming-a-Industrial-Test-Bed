import numpy as np

from ids_attacks.library import AttackSchedule, make_attack
from ids_experiments.scenario import parse_scenario
from ot_bridge.attacker_proxy import ProxyCore
from ot_bridge.modbus_attacks import (attacker_from_scenario, BlockAttack, ModbusAttacker,
                                      PayloadPoison)
from ot_bridge.modbus_io import LocalStore
from ot_bridge.modbus_map import (CellCommand, COIL_SAFE_STOP, COMMAND_REGISTERS, KIND_MOVE,
                                  REG_CONTEXT)

HOME = CellCommand(step=1, kind=KIND_MOVE, q_target=np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0]))


def test_setpoint_step_poisons_one_joint_in_the_window():
    schedule = AttackSchedule([make_attack(1, 'step', [3], magnitude=0.02, start=2.0,
                                           duration=4.0)])
    attacker = ModbusAttacker(schedule=schedule)
    before, _ = attacker.transform(1.0, HOME.to_registers())
    during, truth = attacker.transform(3.0, HOME.to_registers())
    assert abs(CellCommand.from_registers(before).q_target[3] - (-1.2)) <= 1e-4
    assert abs(CellCommand.from_registers(during).q_target[3] - (-1.18)) <= 1e-4
    assert truth.attack_active and truth.kind == 'step'


def test_payload_poison_flips_the_context_register():
    attacker = ModbusAttacker(payload=PayloadPoison(9, 2.0, 4.0, 1))
    during, truth = attacker.transform(3.0, HOME.to_registers())
    after, _ = attacker.transform(7.0, HOME.to_registers())
    assert during[REG_CONTEXT] == 1 and truth.kind == 'payload_poison'
    assert after[REG_CONTEXT] == 0


def test_block_freeze_holds_a_constant_block():
    attacker = ModbusAttacker(block=BlockAttack(3, 'freeze', 2.0, 4.0))
    attacker.transform(1.9, HOME.to_registers())
    moving_a = CellCommand(step=5, q_target=np.full(7, 0.1)).to_registers()
    moving_b = CellCommand(step=6, q_target=np.full(7, 0.9)).to_registers()
    first, _ = attacker.transform(2.5, moving_a)
    second, _ = attacker.transform(3.5, moving_b)
    assert first == second


def test_scenario_builds_a_combined_attacker():
    scenario = parse_scenario({
        'name': 'mix', 'duration': 180.0,
        'attacks': [{'kind': 'step', 'joint': 4, 'magnitude': 0.02, 'start': 120.0,
                     'duration': 10.0}],
        'modbus_attacks': [{'kind': 'payload_poison', 'start': 60.0, 'duration': 5.0,
                            'value': 1}],
        'attacker': {'role': 'mitm', 'credentials': 'stolen'}})
    attacker = attacker_from_scenario(scenario)
    step, _ = attacker.transform(121.0, HOME.to_registers())
    payload, _ = attacker.transform(61.0, HOME.to_registers())
    assert abs(CellCommand.from_registers(step).q_target[3] - (-1.18)) <= 1e-4
    assert payload[REG_CONTEXT] == 1


def test_proxy_forwards_a_poisoned_block_and_passes_safe_stop():
    schedule = AttackSchedule([make_attack(1, 'step', [3], magnitude=0.02, start=2.0,
                                           duration=4.0)])
    proxy = ProxyCore(ModbusAttacker(schedule=schedule))
    upstream, downstream = LocalStore(), LocalStore()
    upstream.write_holding(0, HOME.to_registers())
    upstream.write_coil(COIL_SAFE_STOP, True)
    truth = proxy.refresh(3.0, upstream, downstream)
    forwarded = CellCommand.from_registers(downstream.read_holding(0, COMMAND_REGISTERS))
    assert abs(forwarded.q_target[3] - (-1.18)) <= 1e-4 and truth.attack_active
    assert downstream.read_coil(COIL_SAFE_STOP)
    