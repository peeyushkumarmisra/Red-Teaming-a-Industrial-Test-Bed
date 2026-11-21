
from datetime import datetime
import os

import pytest
import yaml

import signal

from ids_bringup.runs import (BAG_TOPICS, completed_run, controllers_for_run, gazebo_processes,
                              prepare_run, run_id, stop_gazebo)
from ids_experiments.scenario import load_scenario

NOW = datetime(2026, 11, 19, 8, 30, 5)
TEMPLATE = {
    'controller_manager': {'ros__parameters': {'update_rate': 1000}},
    'reference_tracking_controller': {'ros__parameters': {'noise_seed': 1,
                                                          'halt_on_alarm': False}},
}
SCENARIO = {'name': 'step_a4', 'seed': 3, 'friction_profile': 'wear', 'duration': 180.0,
            'planner': {'cycles': 0, 'payload_mass': 5.0},
            'attacks': [{'id': 1, 'kind': 'step', 'joint': 4, 'magnitude': 0.02,
                         'start': 120.0, 'duration': 10.0}]}


@pytest.fixture
def template(tmp_path):
    path = tmp_path / 'controllers.yaml'
    path.write_text(yaml.safe_dump(TEMPLATE))
    return str(path)


def write_summary(directory, status):
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, 'summary.yaml'), 'w') as f:
        yaml.safe_dump({'status': status}, f)


def test_run_id_sorts_by_time_and_names_scenario_and_seed():
    assert run_id('step_a4', 3, NOW) == '20261119-083005_step_a4_s3'


def test_controllers_for_run_sets_seed_and_halt_without_touching_the_template():
    config = controllers_for_run(TEMPLATE, 7, True)
    params = config['reference_tracking_controller']['ros__parameters']
    assert params == {'noise_seed': 7, 'halt_on_alarm': True}
    assert TEMPLATE['reference_tracking_controller']['ros__parameters']['noise_seed'] == 1


def test_prepare_run_writes_the_run_files_and_leaves_the_bag_to_rosbag(tmp_path, template):
    files = prepare_run(str(tmp_path / 'runs'), SCENARIO, template, False, NOW)
    assert os.path.basename(files.directory) == '20261119-083005_step_a4_s3'
    s = load_scenario(files.scenario_file)
    assert (s.name, s.seed, s.attacks[0].joints) == ('step_a4', 3, (3,))
    with open(files.controllers_file) as f:
        controllers = yaml.safe_load(f)
    assert controllers['reference_tracking_controller']['ros__parameters']['noise_seed'] == 3
    assert controllers['controller_manager'] == TEMPLATE['controller_manager']
    assert not os.path.exists(files.bag_directory)
    assert files.summary_file == os.path.join(files.directory, 'summary.yaml')
    with pytest.raises(FileExistsError):
        prepare_run(str(tmp_path / 'runs'), SCENARIO, template, False, NOW)


def test_completed_run_needs_a_complete_summary_for_the_exact_scenario_and_seed(tmp_path):
    root = str(tmp_path)
    assert completed_run(os.path.join(root, 'none'), 'step_a1', 1) is None
    write_summary(os.path.join(root, '20261119-080000_step_a1_s1'), 'no_reference')
    assert completed_run(root, 'step_a1', 1) is None
    write_summary(os.path.join(root, '20261119-090000_step_a1_s1'), 'complete')
    write_summary(os.path.join(root, '20261119-100000_x_step_a1_s1'), 'complete')
    write_summary(os.path.join(root, '20261119-110000_step_a11_s1'), 'complete')
    os.makedirs(os.path.join(root, '20261119-120000_step_a1_s1'))
    assert completed_run(root, 'step_a1', 1) == os.path.join(root, '20261119-090000_step_a1_s1')
    assert completed_run(root, 'step_a1', 2) is None


def test_bag_records_the_streams_the_evaluation_needs():
    for topic in ('/plc/reference', '/robot/reference', '/robot/measurement', '/attack/truth',
                  '/ids/status', '/ids_thesis/status', '/sim/friction_truth'):
        assert topic in BAG_TOPICS
    assert '/clock' not in BAG_TOPICS
    assert '/robot/measurement_raw' in BAG_TOPICS
    assert '/joint_states' in BAG_TOPICS and '/ids/alarm' in BAG_TOPICS


def test_covered_runs_move_the_controller_measurements_behind_the_mitm(tmp_path, template):
    files = prepare_run(str(tmp_path / 'runs'), dict(SCENARIO, cover_torque=True), template,
                        False, NOW)
    with open(files.controllers_file) as f:
        params = yaml.safe_load(f)['reference_tracking_controller']['ros__parameters']
    assert params['measurement_topic'] == '/robot/measurement_raw'
    assert load_scenario(files.scenario_file).cover_torque
    assert params['reference_topic'] == '/robot/reference'
    plain = controllers_for_run(TEMPLATE, 1, False)
    assert 'measurement_topic' not in plain['reference_tracking_controller']['ros__parameters']


@pytest.mark.parametrize('change,topic,halt', [
    ({}, '/robot/reference', False),
    ({'attacks': []}, '/plc/reference', False),
    ({'attacker': {'role': 'rogue', 'credentials': 'none'}}, '/plc/reference', False),
    ({'attacker': {'role': 'spoof'}}, '/plc/reference', False),
    ({'halt_on_alarm': True}, '/robot/reference', True),
])
def test_the_robot_listens_through_the_implant_only_when_on_path(tmp_path, template, change,
                                                                 topic, halt):
    files = prepare_run(str(tmp_path / 'runs'), dict(SCENARIO, **change), template, False, NOW)
    with open(files.controllers_file) as f:
        params = yaml.safe_load(f)['reference_tracking_controller']['ros__parameters']
    assert params['reference_topic'] == topic and params['halt_on_alarm'] is halt
    assert 'measurement_topic' not in params


def fake_proc(root, processes):
    for pid, args in processes.items():
        os.makedirs(os.path.join(root, str(pid)))
        if args is not None:
            with open(os.path.join(root, str(pid), 'cmdline'), 'wb') as f:
                f.write(b'\0'.join(a.encode() for a in args) + b'\0')
    os.makedirs(os.path.join(root, 'self'))


def test_gazebo_processes_finds_gz_sim_servers_only(tmp_path):
    root = str(tmp_path)
    fake_proc(root, {
        101: ['ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim', '-r', '-s', 'cell.sdf'],
        102: ['python3', 'x.py', 'sim'],
        103: ['gz', 'topic', '-l'],
        104: ['gz', 'sim', '-g'],
        105: None,
        106: ['gz sim server', '', ''],
        107: ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch', 'ids_bringup',
              'experiment.launch.py'],
        108: ['foogz', 'sim'],
    })
    assert gazebo_processes(root, own_pid=999) == [101, 104, 106]
    assert gazebo_processes(root, own_pid=104) == [101, 106]


def test_stop_gazebo_terminates_then_kills_what_is_left():
    alive = {7: True, 8: True}
    sent = []

    def find():
        return [p for p, up in alive.items() if up]

    def kill(pid, sig):
        sent.append((pid, sig))
        if sig == signal.SIGKILL or pid == 7:
            alive[pid] = False

    waits = []
    assert stop_gazebo(timeout=1.0, settle=2.0, find=find, kill=kill,
                       sleep=waits.append) == [7, 8]
    assert sent == [(7, signal.SIGTERM), (8, signal.SIGTERM), (8, signal.SIGKILL)]
    assert waits == [0.2] * 5 + [2.0] and not find()


def test_stop_gazebo_does_nothing_without_gazebo():
    def kill(pid, sig):
        raise AssertionError('nothing to kill')

    assert stop_gazebo(find=lambda: [], kill=kill, sleep=None) == []