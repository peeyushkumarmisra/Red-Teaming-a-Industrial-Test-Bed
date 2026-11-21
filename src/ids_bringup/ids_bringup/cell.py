import datetime
import os
import time

import yaml

CELL_DIR = '/ws/data/cell'
RUN_FILE = 'run.yaml'
ROLES = ('plc', 'attacker')
MONITOR_DOMAIN = 20
HEARTBEAT_AGE = 5.0
PLC_REFERENCE = '/plc/reference'
ROBOT_REFERENCE = '/robot/reference'
COVERED_MEASUREMENT = '/robot/measurement_raw'


def write_yaml(path, data):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    os.replace(tmp, path)


def read_yaml(path):
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError):
        return None
    return data if isinstance(data, dict) else None


def attacker_of(scenario):
    spec = scenario.get('attacker') or {}
    role = spec.get('role', 'mitm' if scenario.get('attacks') else 'none')
    default = 'none' if role == 'rogue' else 'stolen'
    return {'role': role, 'credentials': spec.get('credentials', default)}


def wiring(scenario, halt_on_alarm=False):
    on_path = attacker_of(scenario)['role'] == 'mitm'
    return {'reference_topic': ROBOT_REFERENCE if on_path else PLC_REFERENCE,
            'measurement_topic': COVERED_MEASUREMENT if scenario.get('cover_torque') else None,
            'halt_on_alarm': bool(halt_on_alarm or scenario.get('halt_on_alarm', False))}


def roles_for(scenario, mirror_domain=MONITOR_DOMAIN):
    planner = scenario.get('planner') or {}
    return {'plc': {'cycles': int(planner.get('cycles', 0)),
                    'payload_mass': float(planner.get('payload_mass', 5.0)),
                    'mirror_domain': int(mirror_domain)},
            'attacker': attacker_of(scenario)}


def required_roles(scenario):
    return ['plc'] + (['attacker'] if attacker_of(scenario)['role'] != 'none' else [])


def run_path(cell_dir):
    return os.path.join(cell_dir, RUN_FILE)


def report_dir(cell_dir, run_id):
    return os.path.join(cell_dir, 'reports', run_id)


def publish_run(cell_dir, run_id, scenario, roles, now=None):
    data = {'run_id': run_id, 'state': 'running',
            'written': (now or datetime.datetime.now()).isoformat(timespec='seconds'),
            'roles': roles, 'scenario': scenario}
    write_yaml(run_path(cell_dir), data)
    return data


def read_run(cell_dir):
    data = read_yaml(run_path(cell_dir))
    if not data or not isinstance(data.get('run_id'), str) or data.get('state') not in (
            'running', 'finished'):
        return None
    return data


def close_run(cell_dir, run_id=None):
    data = read_run(cell_dir)
    if data is None or data['state'] != 'running':
        return False
    if run_id is not None and data['run_id'] != run_id:
        return False
    data['state'] = 'finished'
    write_yaml(run_path(cell_dir), data)
    return True


def heartbeat_path(cell_dir, role):
    return os.path.join(cell_dir, 'agents', f'{role}.yaml')


def write_heartbeat(cell_dir, role, run_id, pid, now=None):
    write_yaml(heartbeat_path(cell_dir, role),
               {'role': role, 'pid': int(pid), 'run_id': run_id,
                'time': float(time.time() if now is None else now)})


def agent_age(cell_dir, role, now=None):
    data = read_yaml(heartbeat_path(cell_dir, role))
    if not data or 'time' not in data:
        return None
    return (time.time() if now is None else now) - float(data['time'])


def missing_agents(cell_dir, scenario, now=None, max_age=HEARTBEAT_AGE):
    missing = []
    for role in required_roles(scenario):
        age = agent_age(cell_dir, role, now)
        if age is None or age > max_age:
            missing.append(role)
    return missing