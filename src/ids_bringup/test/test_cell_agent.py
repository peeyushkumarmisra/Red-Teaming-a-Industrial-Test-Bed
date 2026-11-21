import os
import signal

import pytest
import yaml

from ids_bringup import cell
from ids_bringup.campaign import load_campaign, resolve
from ids_bringup.cell_agent import (Agent, attacker_processes, credentials_env, main,
                                    parse_args, plc_processes, process_env)

HERE = os.path.dirname(os.path.abspath(__file__))
CAMPAIGN = load_campaign(os.path.join(HERE, '..', 'config', 'campaign.yaml'))


def run_for(name, run_id='r1'):
    scenario = resolve(CAMPAIGN, name, 1)
    return {'run_id': run_id, 'state': 'running', 'roles': cell.roles_for(scenario),
            'scenario': scenario}


def params(command):
    return dict(command[i + 1].split(':=', 1) for i, a in enumerate(command) if a == '-p')


def test_plc_runs_the_planner_with_the_mirror(tmp_path):
    [p] = plc_processes(run_for('nominal'), str(tmp_path))
    assert p.command[:4] == ['ros2', 'run', 'ids_experiments', 'plc_planner']
    assert params(p.command) == {'use_sim_time': 'true', 'cycles': '0', 'payload_mass': '5.0',
                                 'mirror_domain': '20'}
    assert p.env == {} and p.unset == ()


@pytest.mark.parametrize('name,executable,credentials', [
    ('step_a4', 'mitm', 'stolen'),
    ('rogue_nocred', 'rogue', 'none'),
    ('rogue_forged', 'rogue', 'forged'),
    ('joint_states_spoof', 'joint_states_spoof', 'stolen'),
])
def test_attacker_runs_the_scenario_role_with_its_credentials(tmp_path, name, executable,
                                                              credentials):
    [p] = attacker_processes(run_for(name), str(tmp_path))
    assert p.command[:4] == ['ros2', 'run', 'ids_experiments', executable]
    values = params(p.command)
    with open(values['scenario_file']) as f:
        assert yaml.safe_load(f)['name'] == name
    assert values['report_file'] == os.path.join(str(tmp_path), f'{p.name}.yaml')
    assert ('use_sim_time' in values) == (executable == 'mitm')
    assert (p.env, p.unset) == credentials_env(credentials)


def test_no_attacker_means_no_process(tmp_path):
    assert attacker_processes(run_for('nominal'), str(tmp_path)) == []


def test_credentials_become_sros2_settings():
    base = {'ROS_SECURITY_ENABLE': 'true', 'ROS_SECURITY_ENCLAVE_OVERRIDE': '/attacker/implant',
            'KEEP': '1'}
    env, unset = credentials_env('none')
    from ids_bringup.cell_agent import Process
    off = process_env(Process('x', [], env, unset), base)
    assert off == {'ROS_SECURITY_ENABLE': 'false', 'KEEP': '1'}
    env, unset = credentials_env('forged')
    forged = process_env(Process('x', [], env, unset), base)
    assert forged['ROS_SECURITY_ENCLAVE_OVERRIDE'] == '/attacker/forged'
    assert credentials_env('stolen')[0]['ROS_SECURITY_ENCLAVE_OVERRIDE'] == '/attacker/implant'
    with pytest.raises(ValueError):
        credentials_env('borrowed')


class FakeProc:
    def __init__(self, command, env, start_new_session, registry):
        self.command, self.env = command, env
        self.pid = 100 + len(registry)
        self.returncode = None
        self.stubborn = registry.stubborn
        registry.append(self)

    def poll(self):
        return self.returncode


class Registry(list):
    stubborn = 0


class World:
    def __init__(self, stubborn=0):
        self.t = 0.0
        self.procs = Registry()
        self.procs.stubborn = stubborn
        self.signals = []
        self.lines = []

    def popen(self, command, env, start_new_session):
        assert start_new_session
        return FakeProc(command, env, start_new_session, self.procs)

    def killpg(self, pid, sig):
        self.signals.append((pid, sig))
        proc = next(p for p in self.procs if p.pid == pid)
        if sig == signal.SIGKILL or (sig == signal.SIGINT and not proc.stubborn) or \
                (sig == signal.SIGTERM and proc.stubborn < 2):
            proc.returncode = -int(sig)

    def clock(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def agent(tmp_path, role, world, **kw):
    return Agent(role, str(tmp_path), popen=world.popen, killpg=world.killpg, clock=world.clock,
                 sleep=world.sleep, log=world.lines.append, base_env={'PATH': '/bin'}, **kw)


def test_agent_follows_the_run_file(tmp_path):
    world = World()
    a = agent(tmp_path, 'attacker', world)
    a.step()
    assert a.current is None and world.procs == []
    scenario = resolve(CAMPAIGN, 'rogue_nocred', 1)
    cell.publish_run(str(tmp_path), 'r1', scenario, cell.roles_for(scenario))
    a.step()
    assert a.current == 'r1' and len(world.procs) == 1
    proc = world.procs[0]
    assert proc.command[3] == 'rogue' and proc.env['ROS_SECURITY_ENABLE'] == 'false'
    a.step()
    assert len(world.procs) == 1
    with open(cell.heartbeat_path(str(tmp_path), 'attacker')) as f:
        assert yaml.safe_load(f)['run_id'] == 'r1'
    cell.close_run(str(tmp_path), 'r1')
    a.step()
    assert a.current is None and world.signals == [(proc.pid, signal.SIGINT)]
    record = yaml.safe_load(open(os.path.join(cell.report_dir(str(tmp_path), 'r1'),
                                              'attacker_agent.yaml')))
    assert record['event'] == 'stopped' and record['processes'][0]['name'] == 'rogue'


def test_a_new_run_replaces_the_old_one(tmp_path):
    world = World()
    a = agent(tmp_path, 'plc', world)
    nominal = resolve(CAMPAIGN, 'nominal', 1)
    cell.publish_run(str(tmp_path), 'r1', nominal, cell.roles_for(nominal))
    a.step()
    cell.publish_run(str(tmp_path), 'r2', nominal, cell.roles_for(nominal))
    a.step()
    assert a.current == 'r2' and len(world.procs) == 2
    assert world.procs[0].returncode is not None and world.procs[1].returncode is None


@pytest.mark.parametrize('stubborn,expected', [
    (1, [signal.SIGINT, signal.SIGTERM]),
    (2, [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]),
])
def test_stop_escalates_when_a_process_ignores_sigint(tmp_path, stubborn, expected):
    world = World(stubborn=stubborn)
    a = agent(tmp_path, 'plc', world, stop_timeout=1.0)
    nominal = resolve(CAMPAIGN, 'nominal', 1)
    cell.publish_run(str(tmp_path), 'r1', nominal, cell.roles_for(nominal))
    a.step()
    a.stop()
    assert [s for _, s in world.signals] == expected
    assert world.procs[0].returncode is not None


def test_exits_are_logged_once_and_bad_runs_start_nothing(tmp_path):
    world = World()
    a = agent(tmp_path, 'plc', world)
    nominal = resolve(CAMPAIGN, 'nominal', 1)
    cell.publish_run(str(tmp_path), 'r1', nominal, cell.roles_for(nominal))
    a.step()
    world.procs[0].returncode = 1
    a.step()
    a.step()
    assert sum('exited with 1' in line for line in world.lines) == 1
    cell.publish_run(str(tmp_path), 'r2', nominal, {'plc': {}})
    a.step()
    assert a.current == 'r2' and len(world.procs) == 1
    assert any('cannot plan' in line for line in world.lines)


def test_run_loop_and_arguments(tmp_path):
    world = World()
    a = agent(tmp_path, 'plc', world)

    class Once:
        calls = 0

        def is_set(self):
            self.calls += 1
            return self.calls > 2

        def wait(self, period):
            pass

    a.run(Once())
    assert world.lines[0].startswith('[plc] agent watching') and world.lines[-1].endswith(
        'agent stopped')
    assert parse_args(['attacker', '--cell-dir', '/x', '--ros-args']).cell_dir == '/x'
    with pytest.raises(ValueError):
        Agent('hmi', str(tmp_path))
    assert callable(main)