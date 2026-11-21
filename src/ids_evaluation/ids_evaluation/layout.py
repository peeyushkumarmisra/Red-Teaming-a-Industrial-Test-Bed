import numpy as np

DOF = 7
KINDS = ('step', 'ramp', 'sine', 'scaling', 'freeze', 'replay', 'delay', 'noise', 'coordinated',
         'drift_mimic')
INSTANCES = {'v2': '/ids/status', 'thesis': '/ids_thesis/status'}

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


def pad_status(status):
    s = np.asarray(status, dtype=float)
    if s.ndim != 2 or s.shape[1] not in (STATUS_COLUMNS - 1, STATUS_COLUMNS):
        raise ValueError(f'status arrays need {STATUS_COLUMNS} columns')
    if s.shape[1] == STATUS_COLUMNS:
        return s
    return np.hstack([s, np.full((len(s), 1), np.nan)])