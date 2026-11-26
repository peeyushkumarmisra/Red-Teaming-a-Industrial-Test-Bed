import numpy as np

from ids_experiments.kinematics import Kinematics, station_targets
from ids_experiments.sequencer import shuttle_program
from ids_experiments.trajectory import move_duration
from ot_bridge.modbus_io import write_command
from ot_bridge.modbus_map import (CellCommand, COIL_IDS_ALARM, COIL_SAFE_STOP, KIND_DWELL)
from ot_bridge.soft_sequencer import SoftSequencer

VELOCITY_LIMITS = np.array([1.4834, 1.4834, 1.7452, 1.3089, 2.2688, 2.356, 2.356])


def build_program(start, stations, payload_mass=5.0, travel_time=4.0, approach_time=2.0,
                  dwell=0.5, speed_scale=0.5, max_acceleration=2.0, places=None):
    limits = VELOCITY_LIMITS * speed_scale
    accel = np.full(7, max_acceleration)
    travel = max(travel_time,
                 move_duration(stations['a']['hover'], stations['b']['hover'], limits, accel),
                 move_duration(start, stations['a']['hover'], limits, accel))
    approach = max(approach_time,
                   move_duration(stations['a']['hover'], stations['a']['grab'], limits, accel))
    return shuttle_program(stations['a'], stations['b'], payload_mass, travel, approach, dwell,
                           places)


def stations_from_urdf(urdf, start, source, destination, hover_height, grab_height, drop_height):
    kinematics = Kinematics(urdf)
    targets = (station_targets('a', source, hover_height, grab_height, drop_height)
               + station_targets('b', destination, hover_height, grab_height, drop_height))
    solutions = kinematics.chain(targets, np.asarray(start, dtype=float))
    return {s: {k: solutions[f'{s}_{k}'] for k in ('hover', 'grab', 'drop')}
            for s in ('a', 'b')}


class PlcCore:

    def __init__(self, start, program, passes=0, scan_period=0.01):
        self._start = np.asarray(start, dtype=float).copy()
        self._sequencer = SoftSequencer(start, program, passes=passes)
        self._dt = scan_period
        self._clock = 0.0
        self._scan_seq = 0
        self._paused = False
        self._gripper = 0
        self._last = CellCommand(step=0, kind=KIND_DWELL, q_target=self._start.copy())

    @property
    def paused(self):
        return self._paused

    @property
    def clock(self):
        return self._clock

    @property
    def finished(self):
        return self._sequencer.finished

    def scan(self, store):
        self._scan_seq = (self._scan_seq + 1) & 0xFFFF
        alarm = store.read_coil(COIL_IDS_ALARM)
        if alarm:
            self._paused = True
            command = CellCommand(seq=self._scan_seq, step=self._last.step, kind=KIND_DWELL,
                                  duration_ms=0, context=self._last.context,
                                  gripper=self._gripper,
                                  passes=self._sequencer.completed_passes,
                                  finished=int(self._sequencer.finished),
                                  q_target=np.asarray(self._last.q_target, dtype=float).copy())
            store.write_coil(COIL_SAFE_STOP, True)
        else:
            self._paused = False
            self._clock += self._dt
            command, _seat = self._sequencer.scan(self._clock)
            command.seq = self._scan_seq
            self._gripper = command.gripper
            self._last = command
            store.write_coil(COIL_SAFE_STOP, False)
        write_command(store, command)
        return command


def run_server(core, host='0.0.0.0', port=502, scan_period=0.01):
    import threading
    import time

    from pymodbus.datastore import ModbusServerContext
    from ot_bridge.modbus_io import ContextStore, new_context

    context = new_context()
    store = ContextStore(context)
    store.write_coil(0, True)
    try:
        server_context = ModbusServerContext(slaves=context, single=True)
    except TypeError:
        server_context = ModbusServerContext(devices=context, single=True)
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            core.scan(store)
            time.sleep(scan_period)

    worker = threading.Thread(target=loop, daemon=True)
    worker.start()
    import asyncio

    from pymodbus.server import StartAsyncTcpServer
    try:
        asyncio.run(StartAsyncTcpServer(context=server_context, address=(host, port)))
    finally:
        stop.set()
        worker.join(timeout=2.0)
    return store