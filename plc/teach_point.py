import argparse
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'ot_bridge'))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'ids_experiments'))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'ids_attacks'))

from ids_experiments.sequencer import Command, Dwell, Move, SetContext  # noqa: E402
from ot_bridge.modbus_map import (CellCommand, decode_angle, DOF, encode_angle,  # noqa: E402
                                  KIND_DWELL, KIND_IDLE, KIND_MOVE)

ATTACH, DETACH, SEAT = 1, 2, 3


def counts(q):
    return [encode_angle(x) for x in q]


def serialize_program(start, program):
    teach = []

    def teach_index(target):
        key = counts(target)
        if key not in teach:
            teach.append(key)
        return teach.index(key)

    steps = []
    for step in program:
        row = {'kind': KIND_IDLE, 'teach': -1, 'duration_ms': 0, 'context': -1, 'gripper': 0}
        if isinstance(step, Move):
            row.update(kind=KIND_MOVE, teach=teach_index(step.target),
                       duration_ms=int(round(step.duration * 1000.0)))
        elif isinstance(step, Dwell):
            row.update(kind=KIND_DWELL, duration_ms=int(round(step.duration * 1000.0)))
        elif isinstance(step, SetContext):
            row['context'] = int(step.context)
        elif isinstance(step, Command):
            row['gripper'] = {'attach': ATTACH, 'detach': DETACH, 'release': DETACH,
                              'seat': SEAT}.get(step.name, 0)
        steps.append(row)
    return {'start': counts(start), 'teach': teach, 'steps': steps}


def simulate(serialized, scan_period=0.01, passes=1):
    start = np.array([decode_angle(r) for r in serialized['start']], dtype=float)
    teach = [np.array([decode_angle(r) for r in t], dtype=float) for t in serialized['teach']]
    steps = serialized['steps']
    hold = start.copy()
    clock = 0.0
    index = 0
    started = None
    context = 0
    gripper = 0
    completed = 0
    seq = 0
    stream = []
    finished = not steps
    while not finished:
        clock += scan_period
        step = steps[index]
        kind = KIND_IDLE
        target = hold.copy()
        duration_ms = 0
        if started is None:
            started = clock
            if step['gripper'] == ATTACH:
                gripper = 1
            elif step['gripper'] == DETACH:
                gripper = 0
            if step['context'] >= 0:
                context = step['context']
        elapsed = clock - started
        done = True
        if step['kind'] == KIND_MOVE:
            kind = KIND_MOVE
            target = teach[step['teach']]
            duration_ms = step['duration_ms']
            done = elapsed >= step['duration_ms'] / 1000.0 - 1e-9
            if done:
                hold = target.copy()
        elif step['kind'] == KIND_DWELL:
            kind = KIND_DWELL
            done = elapsed >= step['duration_ms'] / 1000.0 - 1e-9
        seq += 1
        stream.append(CellCommand(seq=seq, step=index, kind=kind, duration_ms=duration_ms,
                                  context=context, gripper=gripper, q_target=target.copy()))
        if done:
            started = None
            index += 1
            if index >= len(steps):
                index = 0
                completed += 1
                if passes and completed >= passes:
                    finished = True
    return stream


def render_st(serialized):
    teach = serialized['teach']
    steps = serialized['steps']
    start = serialized['start']
    nt, ns = len(teach), len(steps)
    init = []
    for i, row in enumerate(teach):
        init += [f'      teach[{i},{j}] := {value};' for j, value in enumerate(row)]
    for j, value in enumerate(start):
        init.append(f'      hold[{j}] := {value};')
    for i, step in enumerate(steps):
        init.append(f"      prog_kind[{i}] := {step['kind']}; prog_teach[{i}] := "
                    f"{step['teach']}; prog_ms[{i}] := {step['duration_ms']}; prog_ctx[{i}] := "
                    f"{step['context']}; prog_grip[{i}] := {step['gripper']};")
    init_block = '\n'.join(init)
    return f"""PROGRAM cell_sequencer
  VAR
    initialized : BOOL := FALSE;
    pidx : INT := 0;
    started : BOOL := FALSE;
    start_ms : DINT := 0;
    clock_ms : DINT := 0;
    elapsed_ms : DINT := 0;
    context : INT := 0;
    gripper : INT := 0;
    seq : UINT := 0;
    completed : INT := 0;
    done : BOOL := FALSE;
    first : BOOL := FALSE;
    j : INT := 0;
    hold : ARRAY[0..{DOF - 1}] OF UINT;
    qbuf : ARRAY[0..{DOF - 1}] OF UINT;
    teach : ARRAY[0..{nt - 1}, 0..{DOF - 1}] OF UINT;
    prog_kind : ARRAY[0..{ns - 1}] OF INT;
    prog_teach : ARRAY[0..{ns - 1}] OF INT;
    prog_ms : ARRAY[0..{ns - 1}] OF DINT;
    prog_ctx : ARRAY[0..{ns - 1}] OF INT;
    prog_grip : ARRAY[0..{ns - 1}] OF INT;
  END_VAR
  VAR
    out_seq AT %QW0 : UINT;
    out_step AT %QW1 : UINT;
    out_kind AT %QW2 : UINT;
    out_ms AT %QW3 : UINT;
    out_ctx AT %QW4 : UINT;
    out_grip AT %QW5 : UINT;
    out_passes AT %QW6 : UINT;
    out_finished AT %QW7 : UINT;
    out_q0 AT %QW10 : UINT;
    out_q1 AT %QW11 : UINT;
    out_q2 AT %QW12 : UINT;
    out_q3 AT %QW13 : UINT;
    out_q4 AT %QW14 : UINT;
    out_q5 AT %QW15 : UINT;
    out_q6 AT %QW16 : UINT;
    run_coil AT %QX0.0 : BOOL;
    alarm_coil AT %QX1.0 : BOOL;
    safe_coil AT %QX1.1 : BOOL;
  END_VAR

  IF NOT initialized THEN
{init_block}
      FOR j := 0 TO {DOF - 1} DO qbuf[j] := hold[j]; END_FOR;
      run_coil := TRUE;
      initialized := TRUE;
  END_IF;

  clock_ms := clock_ms + 10;
  seq := seq + 1;

  IF alarm_coil THEN
    safe_coil := TRUE;
  ELSE
    safe_coil := FALSE;
    IF NOT started THEN
      started := TRUE;
      start_ms := clock_ms;
      first := TRUE;
    ELSE
      first := FALSE;
    END_IF;
    IF first THEN
      IF prog_grip[pidx] = 1 THEN gripper := 1;
      ELSIF prog_grip[pidx] = 2 THEN gripper := 0;
      END_IF;
      IF prog_ctx[pidx] >= 0 THEN context := prog_ctx[pidx]; END_IF;
    END_IF;
    elapsed_ms := clock_ms - start_ms;
    done := TRUE;
    out_kind := 0;
    out_ms := 0;
    IF prog_kind[pidx] = 1 THEN
      out_kind := 1;
      out_ms := DINT_TO_UINT(prog_ms[pidx]);
      FOR j := 0 TO {DOF - 1} DO qbuf[j] := teach[prog_teach[pidx], j]; END_FOR;
      IF elapsed_ms < prog_ms[pidx] THEN
        done := FALSE;
      ELSE
        FOR j := 0 TO {DOF - 1} DO hold[j] := teach[prog_teach[pidx], j]; END_FOR;
      END_IF;
    ELSIF prog_kind[pidx] = 2 THEN
      out_kind := 2;
      FOR j := 0 TO {DOF - 1} DO qbuf[j] := hold[j]; END_FOR;
      IF elapsed_ms < prog_ms[pidx] THEN done := FALSE; END_IF;
    ELSE
      FOR j := 0 TO {DOF - 1} DO qbuf[j] := hold[j]; END_FOR;
    END_IF;
    out_step := INT_TO_UINT(pidx);
    IF done THEN
      started := FALSE;
      pidx := pidx + 1;
      IF pidx >= {ns} THEN
        pidx := 0;
        completed := completed + 1;
      END_IF;
    END_IF;
  END_IF;

  out_seq := seq;
  out_ctx := INT_TO_UINT(context);
  out_grip := INT_TO_UINT(gripper);
  out_passes := INT_TO_UINT(completed);
  out_q0 := qbuf[0];
  out_q1 := qbuf[1];
  out_q2 := qbuf[2];
  out_q3 := qbuf[3];
  out_q4 := qbuf[4];
  out_q5 := qbuf[5];
  out_q6 := qbuf[6];
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK Main(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM Inst0 WITH Main : cell_sequencer;
  END_RESOURCE
END_CONFIGURATION
"""


def build_from_urdf(urdf_path, start, source, destination, hover, grab, drop, places,
                    payload_mass, travel_time, approach_time, dwell):
    from ot_bridge.soft_plc import build_program, stations_from_urdf
    with open(urdf_path) as f:
        urdf = f.read()
    stations = stations_from_urdf(urdf, start, source, destination, hover, grab, drop)
    program = build_program(start, stations, payload_mass, travel_time, approach_time, dwell,
                            places)
    teach_yaml = {'start': list(map(float, start)), 'payload_mass': payload_mass,
                  'places': [list(p) for p in places] if places else None,
                  'stations': {s: {k: list(map(float, stations[s][k]))
                                   for k in ('hover', 'grab', 'drop')} for s in ('a', 'b')}}
    return program, teach_yaml


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Offline teach-point generator: solve IK once and emit the PLC teach/program '
                    'tables (ST) and a soft-PLC teach YAML.')
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--out-st', default=os.path.join(HERE, 'cell.st'))
    parser.add_argument('--out-yaml', default=os.path.join(HERE, 'teach.yaml'))
    parser.add_argument('--source', type=float, nargs=2, default=[-0.5, 0.0])
    parser.add_argument('--destination', type=float, nargs=2, default=[0.0, 0.5])
    parser.add_argument('--hover', type=float, default=0.25)
    parser.add_argument('--grab', type=float, default=0.10)
    parser.add_argument('--drop', type=float, default=0.11)
    parser.add_argument('--payload-mass', type=float, default=5.0)
    parser.add_argument('--travel-time', type=float, default=4.0)
    parser.add_argument('--approach-time', type=float, default=2.0)
    parser.add_argument('--dwell', type=float, default=0.5)
    args = parser.parse_args(argv)
    start = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])
    places = (tuple(args.destination), tuple(args.source))
    program, teach_yaml = build_from_urdf(
        args.urdf, start, args.source, args.destination, args.hover, args.grab, args.drop,
        places, args.payload_mass, args.travel_time, args.approach_time, args.dwell)
    serialized = serialize_program(start, program)
    with open(args.out_st, 'w') as f:
        f.write(render_st(serialized))
    with open(args.out_yaml, 'w') as f:
        yaml.safe_dump(teach_yaml, f, sort_keys=False)
    print(f'{len(serialized["teach"])} teach points, {len(serialized["steps"])} program steps '
          f'-> {args.out_st}, {args.out_yaml}')
    return 0


if __name__ == '__main__':
    sys.exit(main())