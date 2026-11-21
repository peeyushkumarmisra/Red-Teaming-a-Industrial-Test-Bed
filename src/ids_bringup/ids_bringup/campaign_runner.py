
import argparse
import os
import subprocess
import time

from ids_bringup.campaign import load_campaign, plan
from ids_bringup.cell import CELL_DIR, close_run
from ids_bringup.runs import completed_run, stop_gazebo


def launch_command(name, seed, output_dir, gui=False, halt_on_alarm=False, cell_dir=CELL_DIR):
    return ['ros2', 'launch', 'ids_bringup', 'experiment.launch.py', f'scenario:={name}',
            f'seed:={seed}', f'output_dir:={output_dir}', f'gui:={str(gui).lower()}',
            f'halt_on_alarm:={str(halt_on_alarm).lower()}', f'cell_dir:={cell_dir}']


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run campaign scenarios one after another (the IDS must already be '
                    'running, armed, in the ids container).')
    parser.add_argument('--campaign', default='', help='campaign file (default: installed)')
    parser.add_argument('--only', nargs='+', default=None,
                        help="scenario names or quoted patterns, e.g. nominal 'step_*'")
    parser.add_argument('--seeds', nargs='+', type=int, default=None)
    parser.add_argument('--output-dir', default='/ws/data/runs')
    parser.add_argument('--rerun', action='store_true', help='repeat completed runs')
    parser.add_argument('--dry-run', action='store_true', help='print the plan only')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--halt-on-alarm', action='store_true')
    parser.add_argument('--pause', type=float, default=5.0,
                        help='seconds to wait between runs')
    parser.add_argument('--cell-dir', default=CELL_DIR,
                        help='run-control folder shared with the cell agents')
    return parser.parse_args(argv)


def default_campaign():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ids_bringup'), 'config', 'campaign.yaml')


def run(args, execute=None, sleep=None, stop=None, close=None):
    execute = execute or subprocess.run
    sleep = sleep or time.sleep
    stop = stop or stop_gazebo
    close = close or close_run
    runs = plan(load_campaign(args.campaign or default_campaign()), args.only, args.seeds)
    results = []
    for i, (name, seed) in enumerate(runs, 1):
        label = f'[{i}/{len(runs)}] {name} seed {seed}'
        done = completed_run(args.output_dir, name, seed)
        if done and not args.rerun:
            print(f'{label}: already complete in {done}', flush=True)
            results.append((name, seed, 'skipped'))
            continue
        command = launch_command(name, seed, args.output_dir, args.gui, args.halt_on_alarm,
                                 args.cell_dir)
        print(f'{label}: {" ".join(command)}', flush=True)
        if args.dry_run:
            results.append((name, seed, 'planned'))
            continue
        execute(command, check=False)
        close(args.cell_dir)
        stop()
        status = 'complete' if completed_run(args.output_dir, name, seed) else 'failed'
        results.append((name, seed, status))
        print(f'{label}: {status}', flush=True)
        if i < len(runs):
            sleep(args.pause)
    failed = [r for r in results if r[2] == 'failed']
    print(f'{len(results)} runs: {len(results) - len(failed)} ok, {len(failed)} failed')
    for name, seed, _ in failed:
        print(f'  failed: {name} seed {seed}')
    return results


def main(argv=None):
    results = run(parse_args(argv))
    return 1 if any(r[2] == 'failed' for r in results) else 0