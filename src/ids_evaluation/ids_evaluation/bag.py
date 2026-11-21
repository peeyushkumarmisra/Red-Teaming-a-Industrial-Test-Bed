
import json
import math
import os

import numpy as np
import yaml

from ids_evaluation import layout, security

TOPICS = ('/robot/measurement', '/robot/measurement_raw', '/plc/reference', '/gripper/attach',
          '/gripper/detach', '/attack/truth', '/sim/friction_truth', '/ids/status',
          '/ids_thesis/status', '/robot/reference', '/joint_states', '/ids/alarm')
FOREIGN_TOPICS = ('/plc/reference', '/robot/reference', '/joint_states')
ROGUE_MARK = 'rogue'
CALIBRATION_FILES = (('v2', 'ids_v2.yaml'), ('thesis', 'ids_thesis.yaml'))


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _yaml(path):
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return yaml.safe_load(f)


def read_limits(calibration_dir):
    limits = {}
    for name, filename in CALIBRATION_FILES:
        data = _yaml(os.path.join(calibration_dir, filename)) if calibration_dir else None
        limits[name] = ((float(data['limit']), float(data['slow_limit'])) if data
                        else (math.nan, math.nan))
    return limits


def read_bag(bag_dir, end=math.inf):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    rows = {k: [] for k in ('measurement', 'measurement_raw', 'reference', 'gripper', 'attack',
                            'friction_truth', 'alarm_topic')}
    foreign = {t: 0 for t in FOREIGN_TOPICS}
    last_alarm = None
    status = {name: ([], []) for name in layout.INSTANCES}
    instance_of = {topic: name for name, topic in layout.INSTANCES.items()}
    last_truth = None
    while reader.has_next():
        topic, data, received = reader.read_next()
        rt = received * 1e-9
        if topic in ('/gripper/attach', '/gripper/detach'):
            if rt <= end:
                rows['gripper'].append([rt, 1.0 if topic.endswith('attach') else 0.0])
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/ids/alarm':
            if rt <= end and bool(msg.data) != last_alarm:
                last_alarm = bool(msg.data)
                rows['alarm_topic'].append([rt, float(msg.data)])
            continue
        if topic in foreign and msg.header.frame_id == ROGUE_MARK:
            foreign[topic] += 1
            continue
        if topic in ('/robot/reference', '/joint_states'):
            continue
        if topic == '/sim/friction_truth':
            values = list(msg.data)
            ft = _stamp(msg) if hasattr(msg, 'header') and msg.header.stamp.sec else rt
            if ft <= end and len(values) >= 14:
                rows['friction_truth'].append([ft, *values[:14]])
            continue
        t = _stamp(msg)
        if t > end:
            continue
        if topic in ('/robot/measurement', '/robot/measurement_raw'):
            rows[topic.rsplit('/', 1)[1]].append(
                [t, msg.reference_seq, float(msg.halted), *msg.position, *msg.velocity,
                 *msg.effort_applied])
        elif topic == '/plc/reference':
            rows['reference'].append([t, msg.seq, msg.payload_context, msg.payload_mass,
                                      *msg.position, *msg.velocity, *msg.acceleration])
        elif topic == '/attack/truth':
            key = (bool(msg.attack_active), int(msg.event_id))
            if key != last_truth:
                last_truth = key
                rows['attack'].append([t, float(msg.attack_active), msg.event_id,
                                       layout.mask_bits(msg.joint_mask), msg.magnitude,
                                       layout.kind_index(msg.kind)])
        else:
            times, values = status[instance_of[topic]]
            times.append(t)
            values.append([msg.mode, msg.alarm, msg.latched, msg.primary_joint,
                           layout.mask_bits(msg.physics_alarm), msg.nis, *msg.disturbance,
                           *msg.physics_statistic, *msg.physics_slow_statistic,
                           *msg.command_statistic, *msg.friction_estimate, msg.limit])
    widths = {'measurement': layout.MEASUREMENT_COLUMNS,
              'measurement_raw': layout.MEASUREMENT_COLUMNS,
              'reference': layout.REFERENCE_COLUMNS, 'gripper': 2,
              'attack': layout.ATTACK_COLUMNS, 'friction_truth': layout.FRICTION_TRUTH_COLUMNS,
              'alarm_topic': 2}
    out = {k: np.array(v, dtype=float).reshape(-1, widths[k]) for k, v in rows.items()}
    out['foreign'] = np.array([foreign[t] if t in types else np.nan for t in FOREIGN_TOPICS],
                              dtype=float)
    for name, (times, values) in status.items():
        out[f'{name}_t'] = np.array(times, dtype=float)
        out[name] = np.array(values, dtype=float).reshape(-1, layout.STATUS_COLUMNS)
    return out


def export_run(run_dir, limit=None, calibration_dir=None):
    summary = _yaml(os.path.join(run_dir, 'summary.yaml'))
    if not summary or summary.get('first_reference') is None:
        raise ValueError(f'{run_dir}: no summary.yaml with a first reference')
    start = float(summary['first_reference'])
    end = start + float(limit) if limit else math.inf
    out = read_bag(os.path.join(run_dir, 'bag'), end)
    out['start'] = np.array(start)
    limits = read_limits(calibration_dir)
    out['limits'] = np.array([*limits['v2'], *limits['thesis']])
    out['summary'] = np.array(json.dumps(summary, default=str))
    out['scenario'] = np.array(json.dumps(_yaml(os.path.join(run_dir, 'scenario.yaml')) or {}))
    out['run'] = np.array(os.path.basename(os.path.normpath(run_dir)))
    out['security'] = np.array(json.dumps(security.read_reports(run_dir), default=str))
    return out


def save_export(path, export):
    np.savez_compressed(path, **export)


def _legacy_attack(attack, scenario, start):
    rows = []
    attacks = sorted((scenario or {}).get('attacks') or [], key=lambda a: a.get('start', 0.0))
    n = 0
    for t, active in np.asarray(attack, dtype=float).reshape(-1, 2):
        if active > 0.5:
            spec = attacks[n] if n < len(attacks) else {}
            n += 1
            joints = spec.get('joints') or ([spec['joint']] if 'joint' in spec else [])
            bits = layout.mask_bits([j + 1 in joints for j in range(layout.DOF)])
            rows.append([t, 1.0, n, bits, float(spec.get('magnitude', 0.0)),
                         layout.kind_index(spec.get('kind', ''))])
        elif rows:
            rows.append([t, 0.0, 0, 0, 0.0, -1.0])
    return np.array(rows, dtype=float).reshape(-1, layout.ATTACK_COLUMNS)


def load_export(path):
    with np.load(path, allow_pickle=False) as data:
        out = {k: data[k] for k in data.files}
    if any('__' in k for k in out):
        raise ValueError(f'{path} holds several runs; use split_legacy_export')
    return normalise(out)


def normalise(export):
    out = dict(export)
    for name in layout.INSTANCES:
        out[name] = layout.pad_status(out.get(name, np.zeros((0, layout.STATUS_COLUMNS))))
        out.setdefault(f'{name}_t', np.zeros(0))
    scenario = json.loads(str(out['scenario'])) if 'scenario' in out else {}
    attack = np.asarray(out.get('attack', np.zeros((0, 2))), dtype=float)
    if attack.size and attack.shape[1] == 2:
        out['attack'] = _legacy_attack(attack, scenario, float(out['start']))
    elif not attack.size:
        out['attack'] = np.zeros((0, layout.ATTACK_COLUMNS))
    for key, width in (('friction_truth', layout.FRICTION_TRUTH_COLUMNS),
                       ('measurement_raw', layout.MEASUREMENT_COLUMNS), ('gripper', 2),
                       ('alarm_topic', 2)):
        out.setdefault(key, np.zeros((0, width)))
    out.setdefault('foreign', np.full(len(FOREIGN_TOPICS), np.nan))
    out.setdefault('security', np.array('{}'))
    out.setdefault('limits', np.full(4, np.nan))
    out.setdefault('summary', np.array('{}'))
    out.setdefault('scenario', np.array('{}'))
    out.setdefault('run', np.array('run'))
    return out


def split_legacy_export(path, scenarios=None):
    with np.load(path, allow_pickle=False) as data:
        flat = {k: data[k] for k in data.files}
    runs = {}
    for key, value in flat.items():
        run, field = key.split('__', 1)
        runs.setdefault(run, {})[field] = value
    out = {}
    for run, export in runs.items():
        export['run'] = np.array(run)
        if scenarios and run in scenarios:
            export['scenario'] = np.array(json.dumps(scenarios[run]))
        out[run] = normalise(export)
    return out