import json
import math

import numpy as np

from ids_evaluation import layout

T, HALTED = 0, 2
Q = slice(3, 10)
V = slice(10, 17)
FIELDS = ('run', 'requested', 'alarm', 'alarm_message', 'first_halt', 'latency', 'held_to_end',
          'halted_s', 'max_speed', 'hold_drift')


def first_alarm(export, name='v2', after=0.0):
    status = export.get(name)
    if status is None or not len(status):
        return math.nan
    t = export[f'{name}_t'] - float(export['start'])
    alarm = (status[:, layout.ALARM] > 0.5) & (status[:, layout.MODE] == layout.ARMED) & \
        (t >= after)
    idx = np.nonzero(alarm)[0]
    return float(t[idx[0]]) if len(idx) else math.nan


def first_alarm_message(export, after):
    rows = np.asarray(export.get('alarm_topic', np.zeros((0, 2))), dtype=float)
    if not len(rows) or math.isnan(after):
        return math.nan
    t = rows[:, 0] - float(export['start'])
    idx = np.nonzero((rows[:, 1] > 0.5) & (t >= after - 0.5))[0]
    return float(t[idx[0]]) if len(idx) else math.nan


def halt_summary(export, settle=0.5):
    m = np.asarray(export['measurement'], dtype=float)
    try:
        requested = bool(json.loads(str(export.get('scenario', '{}'))).get('halt_on_alarm'))
    except ValueError:
        requested = False
    if not len(m):
        return None
    start = float(export['start'])
    t = m[:, T] - start
    halted = m[:, HALTED] > 0.5
    tracking = np.nonzero(~halted)[0]
    if not len(tracking):
        return None
    candidates = np.nonzero(halted & (np.arange(len(t)) > tracking[0]))[0]
    if not requested and not len(candidates):
        return None
    alarm = first_alarm(export)
    row = {'run': str(export['run']), 'requested': requested, 'alarm': alarm,
           'alarm_message': first_alarm_message(export, alarm), 'first_halt': math.nan,
           'latency': math.nan, 'held_to_end': False, 'halted_s': 0.0, 'max_speed': math.nan,
           'hold_drift': math.nan}
    if math.isfinite(alarm):
        candidates = candidates[t[candidates] >= alarm - 1e-9]
    if not len(candidates):
        return row
    k = int(candidates[0])
    run_end = np.nonzero(~halted[k:])[0]
    stop = k + int(run_end[0]) if len(run_end) else len(t)
    hold = slice(k, stop)
    settled = (t[hold] >= t[k] + settle)
    speeds = np.abs(m[hold, V][settled]) if settled.any() else np.zeros((0, layout.DOF))
    row.update({'first_halt': float(t[k]),
                'latency': float(t[k] - alarm) if math.isfinite(alarm) else math.nan,
                'held_to_end': stop == len(t),
                'halted_s': float(t[stop - 1] - t[k]) if stop > k else 0.0,
                'max_speed': float(speeds.max()) if speeds.size else math.nan,
                'hold_drift': float(np.abs(m[hold, Q] - m[k, Q]).max())})
    return row