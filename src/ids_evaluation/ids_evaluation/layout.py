# =============================================================================
#  layout.py - column layouts shared by every module
# -----------------------------------------------------------------------------
#  One place that says which column of an exported array holds what: the IDS status
#  (42 columns, one row per window), the trusted reference (25), the measurement (24),
#  the attack truth (6), the friction truth (15) and the ids_replay output (57).
#  Why it matters: export, replay, metrics and plots must agree exactly, or a latency
#  would be measured on the wrong joint.
# =============================================================================

import numpy as np

DOF = 7
KINDS = ('step', 'ramp', 'sine', 'scaling', 'freeze', 'replay', 'delay', 'noise', 'coordinated',
         'drift_mimic')
# The two live IDS instances recorded in every run.
INSTANCES = {'v2': '/ids/status', 'thesis': '/ids_thesis/status'}

# Status row: mode, alarm, latched, primary joint, alarm mask, NIS, then 7 values each of
#             disturbance, fast, slow and command statistics, friction estimate, and the limit.
MODE, ALARM, LATCHED, PRIMARY, MASK, NIS = 0, 1, 2, 3, 4, 5
DISTURBANCE = slice(6, 13)
FAST = slice(13, 20)
SLOW = slice(20, 27)
COMMAND = slice(27, 34)
FRICTION = slice(34, 41)
LIMIT = 41
STATUS_COLUMNS = 42
ARMED = 2

REFERENCE_COLUMNS = 25
MEASUREMENT_COLUMNS = 24
# Attack truth row: t, active, event id, joint bit mask, magnitude, kind index.
ATTACK_COLUMNS = 6
FRICTION_TRUTH_COLUMNS = 15

R_T, R_MODE, R_VALID, R_ALARM, R_LATCHED, R_PRIMARY, R_ONSET, R_NIS = range(8)
R_DISTURBANCE = slice(8, 15)
R_FAST = slice(15, 22)
R_SLOW = slice(22, 29)
R_COMMAND = slice(29, 36)
R_FRICTION = slice(36, 43)
R_PHYSICS_ALARM = slice(43, 50)
R_COMMAND_ERROR = slice(50, 57)
REPLAY_COLUMNS = 57


def mask_bits(flags):
    return sum(1 << i for i, f in enumerate(flags) if f)


def joints_of(bits):
    return tuple(i for i in range(DOF) if int(bits) >> i & 1)


def joint_label(joints):
    return '+'.join(f'a{j + 1}' for j in joints) if joints else '-'


def kind_index(kind):
    return float(KINDS.index(kind)) if kind in KINDS else -1.0


def kind_name(index):
    i = int(index)
    return KINDS[i] if 0 <= i < len(KINDS) else 'unknown'


# Turn ids_replay output into the status layout, so live and replayed detectors are scored by
# the same code.
def status_from_replay(windows, limit):
    w = np.asarray(windows, dtype=float)
    out = np.zeros((len(w), STATUS_COLUMNS))
    out[:, MODE] = w[:, R_MODE]
    out[:, ALARM] = w[:, R_ALARM]
    out[:, LATCHED] = w[:, R_LATCHED]
    out[:, PRIMARY] = w[:, R_PRIMARY]
    out[:, MASK] = (w[:, R_PHYSICS_ALARM] > 0.5) @ (1 << np.arange(DOF))
    out[:, NIS] = w[:, R_NIS]
    out[:, DISTURBANCE] = w[:, R_DISTURBANCE]
    out[:, FAST] = w[:, R_FAST]
    out[:, SLOW] = w[:, R_SLOW]
    out[:, COMMAND] = w[:, R_COMMAND]
    out[:, FRICTION] = w[:, R_FRICTION]
    out[:, LIMIT] = limit
    return w[:, R_T].copy(), out


# v1.2 exports have 41 columns (no limit); add an unknown limit column.
def pad_status(status):
    s = np.asarray(status, dtype=float)
    if s.ndim != 2 or s.shape[1] not in (STATUS_COLUMNS - 1, STATUS_COLUMNS):
        raise ValueError(f'status arrays need {STATUS_COLUMNS} columns')
    if s.shape[1] == STATUS_COLUMNS:
        return s
    return np.hstack([s, np.full((len(s), 1), np.nan)])
