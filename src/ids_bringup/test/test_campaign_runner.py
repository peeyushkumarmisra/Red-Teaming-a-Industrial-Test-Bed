import os

import yaml

from ids_bringup.campaign_runner import launch_command, main, parse_args, run

CAMPAIGN = {
    'defaults': {'seeds': [1, 2]},
    'scenarios': {'quiet': {'duration': 30.0}},
    'campaign': ['quiet'],
}


def make_campaign(tmp_path):
    path = tmp_path / 'campaign.yaml'
    path.write_text(yaml.safe_dump(CAMPAIGN))
    return str(path), str(tmp_path / 'runs')


def fake_launch(output_dir, outcome):
    calls = []

    def execute(command, check):
        calls.append(command)
        args = dict(a.split(':=', 1) for a in command if ':=' in a)
        name, seed = args['scenario'], args['seed']
        directory = os.path.join(output_dir, f'20261119-0900{len(calls):02d}_{name}_s{seed}')
        os.makedirs(directory)
        with open(os.path.join(directory, 'summary.yaml'), 'w') as f:
            yaml.safe_dump({'status': outcome(name, int(seed))}, f)

    return execute, calls


def test_launch_command_passes_scenario_seed_and_flags():
    assert launch_command('step_a3', 2, '/ws/data/runs', gui=True) == [
        'ros2', 'launch', 'ids_bringup', 'experiment.launch.py', 'scenario:=step_a3',
        'seed:=2', 'output_dir:=/ws/data/runs', 'gui:=true', 'halt_on_alarm:=false']


def test_dry_run_only_prints_the_plan(tmp_path, capsys):
    campaign, out = make_campaign(tmp_path)
    results = run(parse_args(['--campaign', campaign, '--output-dir', out, '--dry-run']),
                  execute=None)
    assert results == [('quiet', 1, 'planned'), ('quiet', 2, 'planned')]
    assert 'scenario:=quiet seed:=2' in capsys.readouterr().out


def test_runs_are_executed_checked_and_completed_ones_skipped(tmp_path):
    campaign, out = make_campaign(tmp_path)
    execute, calls = fake_launch(out, lambda name, seed: 'complete')
    sleeps = []
    args = parse_args(['--campaign', campaign, '--output-dir', out])
    stops = []
    assert run(args, execute, sleeps.append, lambda: stops.append(1)) == [
        ('quiet', 1, 'complete'), ('quiet', 2, 'complete')]
    assert len(calls) == 2 and sleeps == [5.0] and len(stops) == 2
    assert run(args, execute, sleeps.append, lambda: stops.append(1)) == [
        ('quiet', 1, 'skipped'), ('quiet', 2, 'skipped')]
    assert len(stops) == 2
    assert len(calls) == 2


def test_failed_runs_are_reported_and_set_the_exit_code(tmp_path, monkeypatch):
    campaign, out = make_campaign(tmp_path)
    execute, _ = fake_launch(out, lambda name, seed: 'complete' if seed == 1 else 'no_reference')
    args = parse_args(['--campaign', campaign, '--output-dir', out, '--pause', '0'])
    assert run(args, execute, lambda s: None, lambda: [])[1] == ('quiet', 2, 'failed')
    monkeypatch.setattr('ids_bringup.campaign_runner.subprocess.run', execute)
    monkeypatch.setattr('ids_bringup.campaign_runner.time.sleep', lambda s: None)
    monkeypatch.setattr('ids_bringup.campaign_runner.stop_gazebo', lambda: [])
    assert main(['--campaign', campaign, '--output-dir', out, '--seeds', '2', '--rerun']) == 1