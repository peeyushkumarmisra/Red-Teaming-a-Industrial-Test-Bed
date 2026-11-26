import numpy as np

from ot_bridge import modbus_io as mio
from ot_bridge.modbus_map import CellCommand, COIL_IDS_ALARM, COIL_SAFE_STOP, KIND_MOVE


def _command():
    return CellCommand(seq=5, step=3, kind=KIND_MOVE, duration_ms=4000, context=1, gripper=1,
                       q_target=np.array([0.1, -0.2, 0.3, -1.2, 0.0, 0.8, 0.05]))


def test_local_store_command_round_trip():
    store = mio.LocalStore()
    command = _command()
    mio.write_command(store, command)
    back = mio.read_command(store)
    assert back.step == 3 and back.kind == KIND_MOVE and back.context == 1
    assert np.abs(back.q_target - command.q_target).max() <= 1e-4


def test_local_store_coils_and_status():
    store = mio.LocalStore()
    store.write_coil(COIL_IDS_ALARM, True)
    store.write_coil(COIL_SAFE_STOP, False)
    assert store.read_coil(COIL_IDS_ALARM) and not store.read_coil(COIL_SAFE_STOP)
    mio.write_status(store, halted=True, step_ack=7, reference_seq=123)
    status = mio.read_status(store)
    assert status['halted'] and status['step_ack'] == 7 and status['reference_seq'] == 123


def test_context_store_matches_local_store():
    local = mio.LocalStore()
    context = mio.ContextStore()
    command = _command()
    mio.write_command(local, command)
    mio.write_command(context, command)
    assert local.read_holding(0, 24) == context.read_holding(0, 24)
    context.write_coil(COIL_SAFE_STOP, True)
    assert context.read_coil(COIL_SAFE_STOP)