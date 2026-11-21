import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ids_evaluation import layout  # noqa: E402

START = 100.0


def status_block(t, alarm=None, primary=None, fast=None, armed_from=0.0, limit=1.0):
    n = len(t)
    s = np.zeros((n, layout.STATUS_COLUMNS))
    s[:, layout.MODE] = np.where(t - START >= armed_from, layout.ARMED, 1)
    if alarm is not None:
        s[:, layout.ALARM] = alarm
    s[:, layout.PRIMARY] = -1 if primary is None else primary
    s[:, layout.FAST] = 0.1 if fast is None else fast
    s[:, layout.SLOW] = 0.05
    s[:, layout.FRICTION] = 1.0
    s[:, layout.LIMIT] = limit
    return s


def attack_rows(events):
    rows = []
    for event_id, kind, joints, magnitude, start, end in events:
        bits = layout.mask_bits([j in joints for j in range(layout.DOF)])
        rows.append([START + start, 1.0, event_id, bits, magnitude, layout.kind_index(kind)])
        rows.append([START + end, 0.0, 0, 0, 0.0, -1.0])
    return np.array(rows, dtype=float).reshape(-1, layout.ATTACK_COLUMNS)


def build_export(duration=60.0, dt=0.01, events=(), alarm_spans=(), primary=3, run='run_s1',
                 seed=0, disturbance_shift=()):
    rng = np.random.default_rng(seed)
    t = START + np.arange(0.0, duration, dt)
    n = len(t)
    alarm = np.zeros(n)
    prim = np.full(n, -1.0)
    for a, b in alarm_spans:
        on = (t - START >= a) & (t - START < b)
        alarm[on] = 1.0
        prim[on] = primary
    v2 = status_block(t, alarm, prim, armed_from=2.0)
    v2[:, layout.DISTURBANCE] = rng.normal(0.0, 1.0, (n, layout.DOF))
    for joint, a, b, shift in disturbance_shift:
        on = (t - START >= a) & (t - START < b)
        v2[on, layout.DISTURBANCE.start + joint] += shift
    thesis = status_block(t, armed_from=2.0)
    reference = np.zeros((n, layout.REFERENCE_COLUMNS))
    reference[:, 0] = t
    reference[:, 1] = np.arange(n)
    measurement = np.zeros((n, layout.MEASUREMENT_COLUMNS))
    measurement[:, 0] = t + 0.001
    return {
        'run': np.array(run), 'start': np.array(START), 'v2_t': t, 'v2': v2,
        'thesis_t': t, 'thesis': thesis, 'reference': reference, 'measurement': measurement,
        'measurement_raw': np.zeros((0, layout.MEASUREMENT_COLUMNS)),
        'gripper': np.zeros((0, 2)), 'attack': attack_rows(events),
        'friction_truth': np.zeros((0, layout.FRICTION_TRUTH_COLUMNS)),
        'limits': np.array([1.0, 1.0, 1.0, np.nan]), 'summary': np.array('{}'),
        'scenario': np.array(json.dumps({})),
    }


@pytest.fixture
def make_export():
    return build_export