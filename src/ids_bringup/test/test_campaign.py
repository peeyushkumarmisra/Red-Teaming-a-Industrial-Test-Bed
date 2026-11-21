import copy
import os

import pytest
import yaml

from ids_bringup.campaign import load_campaign, parse_campaign, plan, resolve
from ids_experiments.scenario import parse_scenario

HERE = os.path.dirname(os.path.abspath(__file__))
CAMPAIGN = os.path.join(HERE, '..', 'config', 'campaign.yaml')
SMALL = {
    'defaults': {'friction_profile': 'wear', 'planner': {'payload_mass': 5.0}, 'seeds': [1, 2]},
    'scenarios': {'quiet': {'duration': 50.0}, 'cal': {'friction_profile': 'nominal',
                                                       'duration': 30.0, 'seeds': [1]}},
    'families': {'step': {'duration': 40.0, 'joints': [2, 5],
                          'attack': {'kind': 'step', 'magnitude': 0.02, 'start': 20.0,
                                     'duration': 5.0}}},
    'campaign': ['quiet', 'step'],
}


def sibling_config(package, name):
    path = os.path.join(HERE, '..', '..', package, 'config', name)
    if not os.path.isfile(path):
        pytest.skip(f'{package} sources not next to ids_bringup')
    with open(path) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def campaign():
    return load_campaign(CAMPAIGN)


def test_installed_campaign_runs_nominal_then_every_joint_for_step_and_ramp(campaign):
    expected = ['nominal'] + [f'step_a{j}' for j in range(1, 8)] + \
        [f'ramp_a{j}' for j in range(1, 8)]
    assert list(campaign.order) == expected
    runs = plan(campaign)
    assert len(runs) == 45
    assert runs[:4] == [('nominal', 1), ('nominal', 2), ('nominal', 3), ('step_a1', 1)]


def test_every_scenario_and_seed_is_a_valid_mitm_scenario(campaign):
    for name, entry in campaign.entries.items():
        for seed in entry['seeds']:
            s = parse_scenario(resolve(campaign, name, seed))
            assert s.name == name and s.seed == seed


def test_family_members_attack_their_own_joint_during_drift(campaign):
    wear_delay = sibling_config('iiwa_sim', 'friction.yaml')['wear']['delay']
    for kind in ('step', 'ramp'):
        for joint in range(1, 8):
            s = parse_scenario(resolve(campaign, f'{kind}_a{joint}', 2))
            assert s.friction_profile == 'wear'
            assert len(s.attacks) == 1
            a = s.attacks[0]
            assert (a.kind, a.joints, a.event_id) == (kind, (joint - 1,), 1)
            assert a.start > wear_delay


def test_calibration_and_smoke_runs(campaign):
    cal = resolve(campaign, 'calibration', 1)
    assert cal['friction_profile'] == 'nominal' and cal['attacks'] == []
    ids = sibling_config('ids_ros', 'ids.yaml')['/**']['ros__parameters']
    assert cal['duration'] >= ids['warmup'] + ids['calibration_duration'] + 5.0
    assert campaign.entries['calibration']['seeds'] == (1,)
    assert resolve(campaign, 'smoke', 1)['duration'] == 60.0
    assert 'calibration' not in campaign.order and 'smoke' not in campaign.order


def test_plan_filters_by_pattern_and_overrides_seeds():
    c = parse_campaign(SMALL)
    assert plan(c) == [('quiet', 1), ('quiet', 2), ('step_a2', 1), ('step_a2', 2),
                       ('step_a5', 1), ('step_a5', 2)]
    assert plan(c, only=['step_*'], seeds=[7]) == [('step_a2', 7), ('step_a5', 7)]
    assert plan(c, only=['cal']) == [('cal', 1)]
    with pytest.raises(ValueError):
        plan(c, only=['ramp_*'])


def test_resolve_applies_defaults_and_rejects_unknown_names():
    c = parse_campaign(SMALL)
    s = resolve(c, 'step_a5', 9)
    assert s['seed'] == 9 and s['planner'] == {'payload_mass': 5.0}
    assert s['attacks'] == [{'kind': 'step', 'magnitude': 0.02, 'start': 20.0,
                             'duration': 5.0, 'id': 1, 'joint': 5}]
    with pytest.raises(ValueError, match='unknown scenario'):
        resolve(c, 'missing', 1)


@pytest.mark.parametrize('change,message', [
    (lambda d: d.update(extra=1), 'unknown keys'),
    (lambda d: d['scenarios']['quiet'].update(speed=1), 'unknown keys'),
    (lambda d: d['families']['step'].update(joints=[]), 'joints'),
    (lambda d: d['campaign'].append('ghost'), 'neither'),
    (lambda d: d['families']['step']['attack'].update(start=39.0), 'step_a2'),
    (lambda d: d['scenarios']['quiet'].update(seeds=[]), 'seeds'),
    (lambda d: d['scenarios']['quiet'].pop('duration'), 'quiet'),
])
def test_invalid_campaigns_are_rejected_with_the_offending_name(change, message):
    data = copy.deepcopy(SMALL)
    change(data)
    with pytest.raises(ValueError, match=message):
        parse_campaign(data)


def test_installed_campaign_offers_every_attack_kind_cover_and_sweeps(campaign):
    kinds = {}
    for name in campaign.entries:
        s = parse_scenario(resolve(campaign, name, 1))
        for a in s.attacks:
            kinds.setdefault(a.kind, name)
    assert set(kinds) == {'step', 'ramp', 'sine', 'scaling', 'freeze', 'replay', 'delay',
                          'noise', 'coordinated', 'drift_mimic'}
    cover = parse_scenario(resolve(campaign, 'step_a4_cover', 1))
    assert cover.cover_torque and cover.attacks[0].joints == (3,)
    assert parse_scenario(resolve(campaign, 'coordinated_a2a4', 1)).attacks[0].joints == (1, 3)
    sweep = [n for n in campaign.entries if n.startswith('step_sweep_')]
    assert len(sweep) == 28 and 'step_sweep_a4_m0.0025' in sweep
    assert not any(n.startswith(('step_sweep', 'ramp_sweep', 'sine', 'step_a4_cover'))
                   for n in campaign.order)


def test_sweeps_expand_joints_times_magnitudes_and_can_be_planned():
    data = copy.deepcopy(SMALL)
    data['sweeps'] = {'probe': {'duration': 40.0, 'joints': [3, 6], 'magnitudes': [0.01, 0.02],
                                'seeds': [1, 2, 3],
                                'attack': {'kind': 'step', 'start': 20.0, 'duration': 5.0}}}
    c = parse_campaign(data)
    names = ['probe_a3_m0.01', 'probe_a3_m0.02', 'probe_a6_m0.01', 'probe_a6_m0.02']
    assert all(n in c.entries for n in names)
    runs = plan(c, only=['probe_*'])
    assert len(runs) == 12 and runs[0] == ('probe_a3_m0.01', 1)
    s = parse_scenario(resolve(c, 'probe_a6_m0.02', 2))
    assert s.attacks[0].joints == (5,) and s.attacks[0].magnitude == 0.02
    data['campaign'].append('probe')
    assert len(plan(parse_campaign(data))) == 6 + 12


@pytest.mark.parametrize('change,message', [
    (lambda d: d['sweeps']['probe'].update(magnitudes=[]), 'magnitudes'),
    (lambda d: d['sweeps']['probe'].update(speed=1), 'unknown keys'),
    (lambda d: d['sweeps']['probe']['attack'].update(start=39.0), 'probe_a3'),
])
def test_invalid_sweeps_are_rejected(change, message):
    data = copy.deepcopy(SMALL)
    data['sweeps'] = {'probe': {'duration': 40.0, 'joints': [3], 'magnitudes': [0.01],
                                'attack': {'kind': 'step', 'start': 20.0, 'duration': 5.0}}}
    change(data)
    with pytest.raises(ValueError, match=message):
        parse_campaign(data)