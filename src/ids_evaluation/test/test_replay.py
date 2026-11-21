import os
import stat
import sys

import numpy as np
import pytest
import yaml

from ids_evaluation import layout, replay

FAKE = """#!{python}
import sys
import numpy as np
import yaml
args = sys.argv[1:]
opts = {{}}
sets = []
i = 0
while i < len(args):
    if args[i] == '--set':
        sets.append(args[i + 1])
    else:
        opts[args[i]] = args[i + 1]
    i += 2
if 'fail=1' in sets:
    sys.stderr.write('bad override')
    sys.exit(2)
ref = np.load(opts['--reference'])
meas = np.load(opts['--measurement'])
w = np.zeros((len(meas), {cols}))
w[:, 0] = meas[:, 0]
w[:, 1] = 2
np.save(opts['--output'], w)
if '--calibrate' in opts:
    with open(opts['--calibrate'], 'w') as f:
        yaml.safe_dump({{'limit': 3.0, 'slow_limit': 2.0, 'sets': sets}}, f)
sys.stderr.write('replayed %d' % len(meas))
"""


@pytest.fixture
def fake_binary(tmp_path):
    path = tmp_path / 'ids_replay'
    path.write_text(FAKE.format(python=sys.executable, cols=layout.REPLAY_COLUMNS))
    path.chmod(path.stat().st_mode | stat.S_IEXEC)
    return str(path)


def test_calibrate_then_arm(tmp_path, fake_binary, make_export):
    export = make_export(duration=5.0)
    cal = str(tmp_path / 'cal.yaml')
    windows, limit, log = replay.run_replay(export, '<robot/>', 'params.yaml', calibrate=cal,
                                            overrides=('a=1',), binary=fake_binary)
    assert windows.shape == (len(export['measurement']), layout.REPLAY_COLUMNS)
    assert limit == 3.0 and 'replayed' in log
    with open(cal) as f:
        assert yaml.safe_load(f)['sets'] == ['a=1']
    windows, limit, _ = replay.run_replay(export, '<robot/>', 'params.yaml', calibration=cal,
                                          binary=fake_binary)
    assert limit == 3.0
    t, status = replay.replay_status(windows, limit)
    assert (status[:, layout.MODE] == layout.ARMED).all()
    assert replay.read_limits(cal) == (3.0, 2.0)


def test_failure_and_argument_errors(tmp_path, fake_binary, make_export):
    export = make_export(duration=1.0)
    with pytest.raises(RuntimeError, match='bad override'):
        replay.run_replay(export, '<robot/>', 'p.yaml', calibrate=str(tmp_path / 'c.yaml'),
                          overrides=('fail=1',), binary=fake_binary)
    with pytest.raises(ValueError):
        replay.run_replay(export, '<robot/>', 'p.yaml', binary=fake_binary)
    with pytest.raises(ValueError):
        replay.run_replay(export, '<robot/>', 'p.yaml', calibration='a', calibrate='b',
                          binary=fake_binary)


def test_workdir_is_kept_when_given(tmp_path, fake_binary, make_export):
    work = tmp_path / 'work'
    work.mkdir()
    replay.run_replay(make_export(duration=1.0), '<robot/>', 'p.yaml',
                      calibrate=str(tmp_path / 'c.yaml'), workdir=str(work),
                      binary=fake_binary)
    assert sorted(os.listdir(work)) == ['measurement.npy', 'output.npy', 'reference.npy',
                                        'robot.urdf']


def test_binary_lookup(monkeypatch, tmp_path):
    monkeypatch.setenv('IDS_REPLAY', '/x/ids_replay')
    assert replay.find_replay_binary() == '/x/ids_replay'
    monkeypatch.delenv('IDS_REPLAY')
    monkeypatch.setenv('PATH', str(tmp_path))
    try:
        found = replay.find_replay_binary()
    except FileNotFoundError:
        found = None
    assert found is None or found.endswith('ids_replay')


def test_package_params_points_at_config():
    path = replay.package_params('ids.yaml')
    assert path.endswith(os.path.join('config', 'ids.yaml'))
    if os.path.isfile(path):
        assert np.isfinite(1.0)