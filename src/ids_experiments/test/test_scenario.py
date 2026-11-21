
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
    assert s.attacks[0].joints == (3,)
    assert s.attacks[1].kind == 'ramp'
    assert s.planner == {'cycles': 0, 'payload_mass': 5.0}
    assert s.cover_torque is False


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
    (('attacks', 0, 'kind'), 'teleport'),
    (('attacks', 0, 'kind'), 'replay'),
    (('attacks', 0, 'kind'), 'sine'),
    (('attacks', 0, 'kind'), 'coordinated'),
    (('attacks', 0, 'joints'), [4]),
    (('attacks', 1, 'magnitude'), None),
    (('cover_torque',), 'yes'),
    (('attacks', 0, 'joint'), 0),
    (('attacks', 0, 'joint'), 8),
    (('attacks', 0, 'magnitude'), float('nan')),
    (('attacks', 0, 'start'), 190.0),
    (('attacks', 0, 'duration'), -1.0),
    (('attacks', 0, 'id'), 1),
    (('attacks', 1, 'start'), 175.0),
    (('extra',), 1),
])
def test_invalid_scenarios_are_rejected(path, value):
    with pytest.raises(ValueError):
        parse_scenario(with_change(path, value))


def test_load_from_file(tmp_path):
    p = tmp_path / 's.yaml'
    p.write_text('name: n\nduration: 30\nattacks:\n  - {kind: step, joint: 2, magnitude: 0.01, '
                 'start: 10, duration: 5}\n')
    s = load_scenario(str(p))
    assert s.attacks[0].event_id == 1
    assert s.attacks[0].joints == (1,)


def test_every_attack_kind_and_the_cover_flag_are_accepted():
    data = {'name': 'kinds', 'seed': 4, 'duration': 300.0, 'cover_torque': True, 'attacks': [
        {'kind': 'step', 'joint': 4, 'magnitude': 0.02, 'start': 10, 'duration': 5},
        {'kind': 'ramp', 'joint': 4, 'magnitude': 0.001, 'start': 20, 'duration': 5},
        {'kind': 'sine', 'joint': 4, 'magnitude': 0.01, 'frequency': 0.5, 'start': 30,
         'duration': 5},
        {'kind': 'scaling', 'joint': 4, 'magnitude': 1.05, 'start': 40, 'duration': 5},
        {'kind': 'freeze', 'joint': 4, 'start': 50, 'duration': 5},
        {'kind': 'replay', 'joint': 4, 'lag': 4.0, 'start': 60, 'duration': 5},
        {'kind': 'delay', 'joint': 4, 'lag': 0.2, 'start': 70, 'duration': 5},
        {'kind': 'noise', 'joint': 4, 'magnitude': 0.002, 'start': 80, 'duration': 5},
        {'kind': 'coordinated', 'joints': [2, 4], 'magnitudes': [0.005, -0.005],
         'start': 90, 'duration': 5},
        {'kind': 'drift_mimic', 'joint': 4, 'magnitude': 0.002, 'start': 100,
         'duration': 30},
    ]}
    s = parse_scenario(data)
    assert s.cover_torque is True
    assert [a.kind for a in s.attacks] == ['step', 'ramp', 'sine', 'scaling', 'freeze',
                                           'replay', 'delay', 'noise', 'coordinated',
                                           'drift_mimic']
    assert [a.event_id for a in s.attacks] == list(range(1, 11))
    assert s.attacks[8].joints == (1, 3)
    assert s.attacks[8].magnitude == 0.005
    assert s.attacks[7].seed == 4
    assert s.attacks[5].lag == 4.0


def test_attacker_defaults_follow_the_attacks():
    s = parse_scenario(VALID)
    assert (s.attacker_role, s.credentials, s.halt_on_alarm) == ('mitm', 'stolen', False)
    quiet = parse_scenario(with_change(('attacks',), []))
    assert quiet.attacker_role == 'none'
    rogue = parse_scenario(with_change(('attacker',), {'role': 'rogue'}))
    assert (rogue.attacker_role, rogue.credentials) == ('rogue', 'none')
    spoof = parse_scenario(with_change(('attacker',), {'role': 'spoof', 'credentials': 'forged'}))
    assert (spoof.attacker_role, spoof.credentials) == ('spoof', 'forged')
    assert parse_scenario(with_change(('halt_on_alarm',), True)).halt_on_alarm


@pytest.mark.parametrize('change,message', [
    ((('attacker',), {'role': 'sniffer'}), 'attacker.role'),
    ((('attacker',), {'role': 'mitm', 'credentials': 'borrowed'}), 'attacker.credentials'),
    ((('attacker',), {'role': 'none'}), 'role other than none'),
    ((('attacker',), {'role': 'rogue', 'extra': 1}), 'unknown attacker keys'),
    ((('attacker',), 'mitm'), 'mapping'),
    ((('halt_on_alarm',), 'yes'), 'halt_on_alarm'),
])
def test_invalid_attacker_settings(change, message):
    with pytest.raises(ValueError, match=message):
        parse_scenario(with_change(*change))


def test_roles_need_attacks_and_the_cover_needs_the_mitm():
    data = with_change(('attacks',), [])
    data['attacker'] = {'role': 'rogue'}
    with pytest.raises(ValueError, match='needs at least one attack'):
        parse_scenario(data)
    data = with_change(('cover_torque',), True)
    data['attacker'] = {'role': 'rogue'}
    with pytest.raises(ValueError, match='cover_torque needs the mitm role'):
        parse_scenario(data)