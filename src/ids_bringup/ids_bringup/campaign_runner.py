# ==================================================================================================
# campaign_runner.py - run many experiments unattended
# --------------------------------------------------------------------------------------------------
# ros2 run ids_bringup run_campaign [--only nominal step_a4] [--seeds 1] [--dry-run]
# Runs each (scenario, seed) in its own launch, skips runs that are already complete (so an
# interrupted campaign resumes), clears Gazebo after each run, and reports failures with a non-zero
# exit code. The IDS runs separately (ids container) and restarts itself at each new run.
# ==================================================================================================

import argparse
import os
import subprocess
import time

from ids_bringup.campaign import load_campaign, plan
from ids_bringup.runs import completed_run, stop_gazebo


# The exact 'ros2 launch' command for one run.
def launch_command(name, seed, output_dir, gui=False, halt_on_alarm=False):
    return ['ros2', 'launch', 'ids_bringup', 'experiment.launch.py', f'scenario:={name}',
            f'seed:={seed}', f'output_dir:={output_dir}', f'gui:={str(gui).lower()}',
            f'halt_on_alarm:={str(halt_on_alarm).lower()}']


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Run campaign scenarios one after another '
                                     '(the IDS must already be running, armed, in the ids '
                                     'container).')
    parser.add_argument('--campaign', default='', help='campaign file (default: installed)')
    parser.add_argument('--only', nargs='+', default=None,
                        help="scenario names or quoted patterns, e.g. nominal 'step_*'")
    parser.add_argument('--seeds', nargs='+', type=int, default=None)
    parser.add_argument('--output-dir', default='/ws/data/runs')
    parser.add_argument('--rerun', action='store_true', help='repeat completed runs')
    parser.add_argument('--dry-run', action='store_true', help='print the plan only')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--halt-on-alarm', action='store_true')
    parser.add_argument('--pause', type=float, default=5.0, help='seconds to wait between runs')
    return parser.parse_args(argv)


def default_campaign():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ids_bringup'), 'config', 'campaign.yaml')


# The run loop; execute, sleep and stop can be replaced in tests.
def run(args, execute=None, sleep=None, stop=None):
    execute = execute or subprocess.run
    sleep = sleep or time.sleep
    stop = stop or stop_gazebo
    runs = plan(load_campaign(args.campaign or default_campaign()), args.only, args.seeds)
    results = []
    for i, (name, seed) in enumerate(runs, 1):
        label = f'[{i}/{len(runs)}] {name} seed {seed}'
        done = completed_run(args.output_dir, name, seed)
        if done and not args.rerun:
            print(f'{label}: already complete in {done}', flush=True)
            results.append((name, seed, 'skipped'))
            continue
        command = launch_command(name, seed, args.output_dir, args.gui, args.halt_on_alarm)
        print(f'{label}: {" ".join(command)}', flush=True)
        if args.dry_run: