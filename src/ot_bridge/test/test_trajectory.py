import numpy as np

from ids_experiments.sequencer import Sequencer, shuttle_program
from ot_bridge.modbus_map import CellCommand, Q_SCALE
from ot_bridge.soft_sequencer import SoftSequencer
from ot_bridge.trajectory_generator import TrajectoryGenerator

START = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])


def _station(seed):
    r = np.random.default_rng(seed)
    base = START + r.uniform(-0.4, 0.4, 7)
    return {'hover': base, 'grab': base + r.uniform(-0.1, 0.1, 7),
            'drop': base + r.uniform(-0.1, 0.1, 7)}


def _program():
    return shuttle_program(_station(1), _station(2), 5.0, 4.0, 2.0, 0.5,
                           ((0.0, 0.5), (-0.5, 0.0)))


def test_split_reproduces_the_v16_sequencer_and_replica_is_identical():
    program = _program()
    reference = Sequencer(START, program, passes=1)
    soft = SoftSequencer(START, program, passes=1)
    robot = TrajectoryGenerator(START)
    replica = TrajectoryGenerator(START)
    worst_vs_v16 = 0.0
    worst_replica = 0.0
    for k in range(6000):
        t = k * 0.01
        ref = reference.scan(t)
        command, _ = soft.scan(t)
        pr = robot.step(t, command)
        pi = replica.step(t, command)
        worst_vs_v16 = max(worst_vs_v16, float(np.abs(pr[0] - ref.position).max()),
                           float(np.abs(pr[1] - ref.velocity).max()),
                           float(np.abs(pr[2] - ref.acceleration).max()))
        worst_replica = max(worst_replica, float(np.abs(pr[0] - pi[0]).max()),
                            float(np.abs(pr[1] - pi[1]).max()))
    assert worst_vs_v16 < 1e-9
    assert worst_replica == 0.0
    assert soft.finished and soft.completed_passes == 1


def test_split_through_quantised_registers_is_within_one_lsb():
    program = _program()
    reference = Sequencer(START, program, passes=1)
    soft = SoftSequencer(START, program, passes=1)
    robot = TrajectoryGenerator(START)
    worst = 0.0
    for k in range(6000):
        t = k * 0.01
        ref = reference.scan(t)
        command, _ = soft.scan(t)
        wire = CellCommand.from_registers(command.to_registers())
        position = robot.step(t, wire)[0]
        worst = max(worst, float(np.abs(position - ref.position).max()))
    assert worst <= 1.0 / Q_SCALE + 1e-9