# =============================================================================
#  runs.py - run folders, recording and clean-up
# -----------------------------------------------------------------------------
#  Each run writes /ws/data/runs/<date-time>_<scenario>_s<seed>/ with scenario.yaml,
#  controllers.yaml (this run's seed and halt setting), summary.yaml and bag/ (MCAP).
#  Also stops Gazebo servers left over from an earlier run, which otherwise capture
#  the next run's controllers (B20).
# =============================================================================

import copy
from dataclasses import dataclass
import os
import re
import signal
import time

import yaml

# Where the part is spawned: on the source pad, resting on the ground.
PAYLOAD_POSE = (-0.5, 0.0, 0.05)
RUN_ID = re.compile(r'^\d{8}-\d{6}_(.+)_s(\d+)$')
GZ_SIM = re.compile(r'(^|[\s/])gz sim(\s|$)')
# Everything the evaluation needs: reference before and after the MitM,
# measurements, attack truth, both IDS instances, friction truth, gripper.
BAG_TOPICS = (
    '/plc/reference',
    '/robot/reference',
    '/robot/measurement',
    '/attack/truth',
    '/ids/status',
    '/ids/alarm',
    '/ids_thesis/status',
    '/ids_thesis/alarm',
    '/sim/friction_truth',
    '/gripper/attach',
    '/gripper/detach',
    '/gripper/state',
)


@dataclass(frozen=True)
class RunFiles:
    directory: str
    scenario_file: str
    controllers_file: str
    summary_file: str
    bag_directory: str


def run_id(name, seed, now):
    return f'{now:%Y%m%d-%H%M%S}_{name}_s{seed}'


# Copy the controller settings with this run's noise seed and halt setting.
def controllers_for_run(template, seed, halt_on_alarm):
    config = copy.deepcopy(template)
    params = config['reference_tracking_controller']['ros__parameters']
    params['noise_seed'] = int(seed)
    params['halt_on_alarm'] = bool(halt_on_alarm)
    return config


# Create the run folder and its input files; rosbag creates bag/ itself.
def prepare_run(output_dir, scenario, controllers_template, halt_on_alarm, now):
    directory = os.path.join(output_dir, run_id(scenario['name'], scenario['seed'], now))
    os.makedirs(directory)
    files = RunFiles(directory,
                     os.path.join(directory, 'scenario.yaml'),
                     os.path.join(directory, 'controllers.yaml'),
                     os.path.join(directory, 'summary.yaml'),
                     os.path.join(directory, 'bag'))
    with open(controllers_template) as f:
        template = yaml.safe_load(f)
    with open(files.scenario_file, 'w') as f:
        yaml.safe_dump(scenario, f, sort_keys=False)
    with open(files.controllers_file, 'w') as f:
        yaml.safe_dump(controllers_for_run(template, scenario['seed'], halt_on_alarm), f,
                       sort_keys=False)
    return files


def read_summary(directory):
    path = os.path.join(directory, 'summary.yaml')
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return yaml.safe_load(f)


# A run counts as done only with a complete summary for the same scenario and seed.
def completed_run(output_dir, name, seed):
    if not os.path.isdir(output_dir):
        return None
    for entry in sorted(os.listdir(output_dir), reverse=True):
        directory = os.path.join(output_dir, entry)
        match = RUN_ID.match(entry)
        if match and match.groups() == (name, str(seed)) and os.path.isdir(directory):
            summary = read_summary(directory)
            if summary and summary.get('status') == 'complete':
                return directory
    return None


# Find running `gz sim` servers through /proc (matches the renamed server, B20).
def gazebo_processes(proc_root='/proc', own_pid=None):
    own_pid = os.getpid() if own_pid is None else own_pid
    found = []
    for entry in os.listdir(proc_root):
        if not entry.isdigit() or int(entry) == own_pid:
            continue
        try:
            with open(os.path.join(proc_root, entry, 'cmdline'), 'rb') as f:
                command = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:
            continue
        if GZ_SIM.search(command):
            found.append(int(entry))
    return sorted(found)


def _signal(kill, pids, sig):
    for pid in pids:
        try:
            kill(pid, sig)
        except ProcessLookupError:
            pass


# SIGTERM, then SIGKILL whatever is still running; then wait for it to settle.
def stop_gazebo(timeout=5.0, settle=2.0, find=gazebo_processes, kill=os.kill, sleep=time.sleep):
    pids = find()
    if not pids:
        return []
    _signal(kill, pids, signal.SIGTERM)
    waited = 0.0
    while find() and waited < timeout:
        sleep(0.2)
        waited += 0.2
    _signal(kill, find(), signal.SIGKILL)
    sleep(settle)
    return pids
