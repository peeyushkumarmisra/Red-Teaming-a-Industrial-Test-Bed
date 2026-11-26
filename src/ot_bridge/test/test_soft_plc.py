import numpy as np

from ids_experiments.sequencer import shuttle_program
from ot_bridge.modbus_io import LocalStore, read_command
from ot_bridge.modbus_map import COIL_IDS_ALARM, COIL_SAFE_STOP, COMMAND_REGISTERS, KIND_DWELL
from ot_bridge.soft_plc import PlcCore

START = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])


def _program():
    def station(seed):
        r = np.random.default_rng(seed)
        base = START + r.uniform(-0.4, 0.4, 7)
        return {'hover': base, 'grab': base + r.uniform(-0.1, 0.1, 7),
                'drop': base + r.uniform(-0.1, 0.1, 7)}
    return shuttle_program(station(1), station(2), 5.0, 4.0, 2.0, 0.5, ((0.0, 0.5), (-0.5, 0.0)))


def test_soft_plc_writes_a_fresh_sequence_each_scan():
    store = LocalStore()
    plc = PlcCore(START, _program(), passes=1)
    seqs = []
    for _ in range(50):
        command = plc.scan(store)
        seqs.append(command.seq)
        assert len(store.read_holding(0, COMMAND_REGISTERS)) == COMMAND_REGISTERS
    assert seqs == list(range(1, 51))


def test_alarm_coil_pauses_the_plc_and_asserts_safe_stop():
    store = LocalStore()
    plc = PlcCore(START, _program(), passes=0)
    for _ in range(500):
        plc.scan(store)
    clock_before = plc.clock
    store.write_coil(COIL_IDS_ALARM, True)
    for _ in range(300):
        command = plc.scan(store)
    assert plc.paused
    assert store.read_coil(COIL_SAFE_STOP)
    assert command.kind == KIND_DWELL
    assert abs(plc.clock - clock_before) < 1e-12
    store.write_coil(COIL_IDS_ALARM, False)
    plc.scan(store)
    assert not plc.paused and not store.read_coil(COIL_SAFE_STOP)
    assert plc.clock > clock_before


def test_soft_plc_completes_the_program():
    store = LocalStore()
    plc = PlcCore(START, _program(), passes=1)
    for _ in range(6000):
        plc.scan(store)
    assert plc.finished
    assert read_command(store).finished == 1