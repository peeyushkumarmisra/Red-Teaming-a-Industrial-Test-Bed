import argparse
import json
import os
import signal
import subprocess
import sys
import time

import yaml

from ids_security.latency import overhead, report_lines

SECURE_DOMAIN = 30
PLAIN_DOMAIN = 31
MODES = ('sros2', 'plain')


def mode_env(mode, base=os.environ):
    env = dict(base)
    if mode == 'sros2':
        env['ROS_DOMAIN_ID'] = str(SECURE_DOMAIN)
        env['ROS_SECURITY_ENABLE'] = 'true'
    elif mode == 'plain':
        env['ROS_DOMAIN_ID'] = str(PLAIN_DOMAIN)
        env['ROS_SECURITY_ENABLE'] = 'false'
    else:
        raise ValueError(f'mode must be one of {MODES}')
    return env


def probe_command(role, extra=()):
    return ['ros2', 'run', 'ids_security', 'probe', role, *extra]


def run_pong(popen=subprocess.Popen, wait_forever=None):
    procs = [popen(probe_command('pong'), env=mode_env(m), start_new_session=True)
             for m in MODES]
    print(f'pong running: sros2 on domain {SECURE_DOMAIN}, plain on domain {PLAIN_DOMAIN}; '
          'Ctrl-C to stop', flush=True)
    try:
        (wait_forever or (lambda: [p.wait() for p in procs]))()
    except KeyboardInterrupt:
        pass
    finally:
        for p in procs:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    return 0


def run_ping(out_dir, count, rate, size, run=subprocess.run):
    os.makedirs(out_dir, exist_ok=True)
    results = {}
    for mode in MODES:
        path = os.path.join(out_dir, f'{mode}.json')
        extra = ['--count', str(count), '--rate', str(rate), '--size', str(size), '--out', path]
        print(f'{mode}: {count} samples at {rate:g} Hz', flush=True)
        run(probe_command('ping', extra), env=mode_env(mode), check=False)
        with open(path) as f:
            results[mode] = json.load(f)
        if results[mode].get('error'):
            raise SystemExit(f"{mode}: {results[mode]['error']}")
    lines = report_lines(results['sros2'], results['plain'], size, rate)
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(os.path.join(out_dir, 'overhead.yaml'), 'w') as f:
        yaml.safe_dump({'created': time.strftime('%Y-%m-%dT%H:%M:%S'), 'size': size,
                        'rate': rate, 'results': results,
                        'overhead': overhead(results['sros2'], results['plain'])}, f,
                       sort_keys=False)
    print('\n'.join(lines))
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='SROS2 overhead: run "overhead pong" in the robot container first, then '
                    '"overhead ping" in the plc container.')
    parser.add_argument('role', choices=('ping', 'pong'))
    parser.add_argument('--out', default='/ws/data/cell/overhead')
    parser.add_argument('--count', type=int, default=3000)
    parser.add_argument('--rate', type=float, default=100.0)
    parser.add_argument('--size', type=int, default=256)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.role == 'pong':
        return run_pong()
    return run_ping(args.out, args.count, args.rate, args.size)


if __name__ == '__main__':
    sys.exit(main())