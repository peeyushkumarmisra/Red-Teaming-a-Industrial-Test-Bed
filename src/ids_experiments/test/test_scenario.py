# =============================================================================
#  test_scenario.py - scenario validation
# =============================================================================

import copy

import pytest

from ids_experiments.scenario import load_scenario, parse_scenario

VALID = {
    'name': 'step_a4',
    'seed': 3,
    'friction_profile': 'wear',
    'duration': 200.0,
    'planner': {'cycles': 0, 'payload_mass': 5.0},
    'attacks': [
        {'id': 2, 'kind': 'ramp', 'joint': 6, 'magnitude': 0.0005, 'start': 150.0,
         'duration': 30.0},
        {'id': 1, 'kind': 'step', 'joint': 4, 'magnitude': 0.02, 'start': 100.0,
         'duration': 10.0},
    ],
}


def with_change(path, value):
    data = copy.deepcopy(VALID)
    target = data
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    return data


def test_valid_scenario_is_parsed_sorted_and_zero_based():
    s = parse_scenario(VALID)
    assert s.name == 'step_a4'
    assert s.seed == 3
    assert s.friction_profile == 'wear'
    assert [a.event_id for a in s.attacks] == [1, 2]
    assert s.attacks[0].joint == 3
    assert s.attacks[1].kind == 'ramp'
    assert s.planner == {'cycles': 0, 'payload_mass': 5.0}


def test_defaults_fill_optional_fields():
    s = parse_scenario({'name': 'nominal', 'duration': 60})
    assert s.seed == 1
    assert s.friction_profile == 'wear'
    assert s.attacks == ()
    assert s.planner['payload_mass'] == 5.0


@pytest.mark.parametrize('path,value', [
    (('name',), ''),
    (('seed',), -1),
    (('friction_profile',), 'sticky'),
    (('duration',), 0.0),
    (('planner', 'cycles'), 1.5),
    (('planner', 'payload_mass'), 0.0),
    (('planner', 'speed'), 1.0),
    (('attacks', 0, 'kind'), 'replay'),
    (('attacks', 0, 'joint'), 0),
    (('attacks', 0, 'joint'), 8),
    (('attacks', 0, 'magnitude'), float('nan')),
    (('attacks', 0, 'start'), 190.0),
    (('attacks', 0, 'duration'), -1.0),
    (('attacks', 0, 'id'), 1),
    (('attacks', 1, 'start'), 175.0),
    (('extra',), 1),
])
# Each broken field gives a clear error before any run starts.
def test_invalid_scenarios_are_rejected(path, value):
    with pytest.raises(ValueError):
        parse_scenario(with_change(path, value))


def test_load_from_file(tmp_path):
    p = tmp_path / 's.yaml'
    p.write_text('name: n\nduration: 30\nattacks:\n  - {kind: step, joint: 2, magnitude: 0.01, '
                 'start: 10, duration: 5}\n')
    s = load_scenario(str(p))
    assert s.attacks[0].event_id == 1
    assert s.attacks[0].joint == 1
