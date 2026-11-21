# =============================================================================
#  test_campaign.py - the installed campaign is complete and valid
# =============================================================================

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


# 45 runs in the documented order.
def test_installed_campaign_runs_nominal_then_every_joint_for_step_and_ramp(campaign):
    expected = ['nominal'] + [f'step_a{j}' for j in range(1, 8)] + \
        [f'ramp_a{j}' for j in range(1, 8)]
    assert list(campaign.order) == expected
    runs = plan(campaign)
    assert len(runs) == 45
    assert runs[:4] == [('nominal', 1), ('nominal', 2), ('nominal', 3), ('step_a1', 1)]


# Every run passes the MitM's own validation.
def test_every_scenario_and_seed_is_a_valid_mitm_scenario(campaign):
    for name, entry in campaign.entries.items():
        for seed in entry['seeds']:
            s = parse_scenario(resolve(campaign, name, seed))
            assert s.name == name and s.seed == seed


# Attacks start after wear has begun.
def test_family_members_attack_their_own_joint_during_drift(campaign):
    wear_delay = sibling_config('iiwa_sim', 'friction.yaml')['wear']['delay']
    for kind in ('step', 'ramp'):
        for joint in range(1, 8):
            s = parse_scenario(resolve(campaign, f'{kind}_a{joint}', 2))
            assert s.friction_profile == 'wear'
            assert len(s.attacks) == 1
            a = s.attacks[0]
            assert (a.kind, a.joint, a.event_id) == (kind, joint - 1, 1)
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
