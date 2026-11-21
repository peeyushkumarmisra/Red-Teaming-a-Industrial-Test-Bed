import json
import os

import numpy as np
import pytest
import yaml

from conftest import START
from ids_evaluation import bag, layout


def test_read_limits(tmp_path):
    with open(tmp_path / 'ids_v2.yaml', 'w') as f:
        yaml.safe_dump({'limit': 2.5, 'slow_limit': 1.5}, f)
    limits = bag.read_limits(str(tmp_path))
    assert limits['v2'] == (2.5, 1.5)
    assert all(np.isnan(limits['thesis']))
    assert all(np.isnan(bag.read_limits(None)['v2']))


def test_save_and_load_round_trip(tmp_path, make_export):
    export = make_export(events=[(1, 'step', (3,), 0.02, 30.0, 40.0)])
    path = str(tmp_path / 'export.npz')
    bag.save_export(path, export)
    back = bag.load_export(path)
    assert str(back['run']) == 'run_s1'
    assert back['attack'].shape == (2, layout.ATTACK_COLUMNS)
    assert np.allclose(back['v2'], export['v2'])


def _legacy(tmp_path):
    t = START + np.arange(0.0, 20.0, 0.01)
    flat = {}
    for run, attack in (('nominal_s1', np.zeros((0, 2))),
                        ('step_a4_s1', np.array([[START + 10.0, 1.0], [START + 15.0, 0.0]]))):
        flat[f'{run}__start'] = np.array(START)
        flat[f'{run}__measurement'] = np.zeros((len(t), layout.MEASUREMENT_COLUMNS))
        flat[f'{run}__reference'] = np.zeros((len(t), layout.REFERENCE_COLUMNS))
        flat[f'{run}__gripper'] = np.zeros((0, 2))
        flat[f'{run}__attack'] = attack
        flat[f'{run}__v2_t'] = t
        flat[f'{run}__v2'] = np.zeros((len(t), layout.STATUS_COLUMNS - 1))
        flat[f'{run}__thesis_t'] = t
        flat[f'{run}__thesis'] = np.zeros((len(t), layout.STATUS_COLUMNS - 1))
    path = str(tmp_path / 'runs_export.npz')
    np.savez(path, **flat)
    return path


def test_split_legacy_export_converts_attack(tmp_path):
    path = _legacy(tmp_path)
    scenarios = {'step_a4_s1': {'attacks': [{'kind': 'step', 'joint': 4, 'magnitude': 0.02,
                                             'start': 10.0, 'duration': 5.0}]}}
    runs = bag.split_legacy_export(path, scenarios)
    assert set(runs) == {'nominal_s1', 'step_a4_s1'}
    a = runs['step_a4_s1']['attack']
    assert a.shape == (2, layout.ATTACK_COLUMNS)
    assert list(a[0]) == [START + 10.0, 1.0, 1, 8, 0.02, 0.0]
    assert a[1, 1] == 0.0
    assert runs['nominal_s1']['attack'].shape == (0, layout.ATTACK_COLUMNS)
    assert runs['step_a4_s1']['v2'].shape[1] == layout.STATUS_COLUMNS
    assert json.loads(str(runs['step_a4_s1']['scenario']))['attacks'][0]['joint'] == 4
    with pytest.raises(ValueError):
        bag.load_export(path)


def test_legacy_attack_without_scenario_is_unknown(tmp_path):
    runs = bag.split_legacy_export(_legacy(tmp_path))
    a = runs['step_a4_s1']['attack']
    assert a[0, 3] == 0 and a[0, 5] == -1.0


def test_export_run_needs_summary(tmp_path):
    os.makedirs(tmp_path / 'bag')
    with pytest.raises(ValueError):
        bag.export_run(str(tmp_path))