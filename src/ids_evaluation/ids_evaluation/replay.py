# =============================================================================
#  replay.py - call ids_replay from Python
# -----------------------------------------------------------------------------
#  Writes an export's references and measurements to .npy files, runs the ids_replay
#  binary (from ids_ros) and reads the per-window output back. A calibration can be
#  made from one run and used to arm on others.
# =============================================================================

import os
import shutil
import subprocess
import tempfile

import numpy as np
import yaml

from ids_evaluation import layout


# IDS_REPLAY environment variable, then the installed ids_ros package, then PATH.
def find_replay_binary():
    explicit = os.environ.get('IDS_REPLAY')
    if explicit:
        return explicit
    try:
        from ament_index_python.packages import get_package_prefix
        candidate = os.path.join(get_package_prefix('ids_ros'), 'lib', 'ids_ros', 'ids_replay')
        if os.access(candidate, os.X_OK):
            return candidate
    except Exception:
        pass
    found = shutil.which('ids_replay')
    if found:
        return found
    raise FileNotFoundError('ids_replay not found: build ids_ros or set IDS_REPLAY')


# The installed ids_ros parameter files (ids.yaml, ids_thesis.yaml).
def package_params(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('ids_ros'), 'config', name)
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', '..', 'ids_ros', 'config', name)


def run_replay(export, urdf, params, workdir=None, calibration=None, calibrate=None,
               duration=120.0, warmup=2.0, overrides=(), binary=None):
    if (calibration is None) == (calibrate is None):
        raise ValueError('give exactly one of calibration and calibrate')
    binary = binary or find_replay_binary()
    own = workdir is None
    workdir = workdir or tempfile.mkdtemp(prefix='ids_replay_')
    try:
        paths = {name: os.path.join(workdir, f'{name}.npy')
                 for name in ('reference', 'measurement', 'output')}
        np.save(paths['reference'], np.ascontiguousarray(export['reference'], dtype=np.float64))
        np.save(paths['measurement'],
                np.ascontiguousarray(export['measurement'], dtype=np.float64))
        urdf_path = os.path.join(workdir, 'robot.urdf')
        with open(urdf_path, 'w') as f:
            f.write(urdf)
        cmd = [binary, '--urdf', urdf_path, '--params', params,
               '--reference', paths['reference'], '--measurement', paths['measurement'],
               '--output', paths['output'], '--warmup', str(warmup)]
        if calibration is not None:
            cmd += ['--calibration', calibration]
        else:
            cmd += ['--calibrate', calibrate, '--duration', str(duration)]
        for o in overrides:
            cmd += ['--set', o]
        done = subprocess.run(cmd, capture_output=True, text=True)
        if done.returncode != 0:
            raise RuntimeError(f'ids_replay failed: {done.stderr.strip()}')
        windows = np.load(paths['output'])
        limit = read_limits(calibration or calibrate)[0]
        return windows, limit, done.stderr
    finally:
        if own:
            shutil.rmtree(workdir, ignore_errors=True)


def read_limits(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return float(data['limit']), float(data['slow_limit'])


def replay_status(windows, limit):
    return layout.status_from_replay(windows, limit)
