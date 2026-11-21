import datetime
import os

import pytest
import yaml

from ids_bringup import cell
from ids_bringup.campaign import load_campaign, resolve
from ids_experiments.scenario import parse_scenario

HERE = os.path.dirname(os.path.abspath(__file__))
CAMPAIGN = os.path.join(HERE, '..', 'config', 'campaign.yaml')
NOW = datetime.datetime(2026, 11, 21, 10, 0, 0)


@pytest.fixture(scope='module')
def campaign():
    return load_campaign(CAMPAIGN)


def test_attacker_defaults_agree_with_the_scenario_parser(campaign):
    for name in campaign.entries:
        scenario = resolve(campaign, name, 1)
        parsed = parse_scenario(scenario)
        assert cell.attacker_of(scenario) == {'role': parsed.attacker_role,
                                              'credentials': parsed.credentials}, name


def test_roles_carry_the_planner_settings_and_the_mirror_domain(campaign):
    roles = cell.roles_for(resolve(campaign, 'step_a4', 1))
    assert roles == {'plc': {'cycles': 0, 'payload_mass': 5.0, 'mirror_domain': 20},
                     'attacker': {'role': 'mitm', 'credentials': 'stolen'}}
    assert cell.roles_for(resolve(campaign, 'nominal', 1), 21)['plc']['mirror_domain'] == 21
    assert cell.required_roles(resolve(campaign, 'nominal', 1)) == ['plc']
    assert cell.required_roles(resolve(campaign, 'rogue_nocred', 1)) == ['plc', 'attacker']


def test_wiring(campaign):
    w = cell.wiring(resolve(campaign, 'step_a4_cover', 1))
    assert w == {'reference_topic': '/robot/reference',
                 'measurement_topic': '/robot/measurement_raw', 'halt_on_alarm': False}
    assert cell.wiring(resolve(campaign, 'nominal', 1), halt_on_alarm=True)['halt_on_alarm']
    assert cell.wiring(resolve(campaign, 'joint_states_spoof', 1))['reference_topic'] == \
        '/plc/reference'


def test_run_file_life_cycle(tmp_path, campaign):
    d = str(tmp_path / 'cell')
    assert cell.read_run(d) is None and not cell.close_run(d)
    scenario = resolve(campaign, 'step_a4', 1)
    data = cell.publish_run(d, 'r1', scenario, cell.roles_for(scenario), NOW)
    assert data['written'] == '2026-11-21T10:00:00'
    run = cell.read_run(d)
    assert run['run_id'] == 'r1' and run['state'] == 'running'
    assert run['scenario']['name'] == 'step_a4'
    assert not cell.close_run(d, 'other')
    assert cell.close_run(d, 'r1')
    assert cell.read_run(d)['state'] == 'finished'
    assert not cell.close_run(d)
    assert not os.path.exists(cell.run_path(d) + '.tmp')


@pytest.mark.parametrize('text', ['', 'just text', 'run_id: 3\nstate: running\n',
                                  'run_id: r\nstate: paused\n', '{bad yaml'])
def test_unreadable_run_files_mean_no_run(tmp_path, text):
    (tmp_path / 'run.yaml').write_text(text)
    assert cell.read_run(str(tmp_path)) is None


def test_heartbeats_decide_which_agents_are_missing(tmp_path, campaign):
    d = str(tmp_path)
    attack = resolve(campaign, 'step_a4', 1)
    assert cell.missing_agents(d, attack, now=1000.0) == ['plc', 'attacker']
    cell.write_heartbeat(d, 'plc', None, 42, now=998.0)
    assert cell.missing_agents(d, attack, now=1000.0) == ['attacker']
    cell.write_heartbeat(d, 'attacker', 'r1', 43, now=990.0)
    assert cell.missing_agents(d, attack, now=1000.0) == ['attacker']
    assert cell.missing_agents(d, resolve(campaign, 'nominal', 1), now=1000.0) == []
    assert cell.agent_age(d, 'plc', now=1000.0) == pytest.approx(2.0)
    with open(cell.heartbeat_path(d, 'attacker')) as f:
        assert yaml.safe_load(f) == {'role': 'attacker', 'pid': 43, 'run_id': 'r1',
                                     'time': 990.0}
    assert cell.report_dir(d, 'r1') == os.path.join(d, 'reports', 'r1')