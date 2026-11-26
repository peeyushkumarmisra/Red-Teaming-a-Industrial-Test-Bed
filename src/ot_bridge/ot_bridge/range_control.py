import argparse
import os
import signal
import subprocess
import sys
import time


def start_pcap(interface, out_path, snaplen=512):
    try:
        return subprocess.Popen(
            ['tcpdump', '-i', interface, '-s', str(snaplen), '-w', out_path, 'tcp port 502'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    except FileNotFoundError:
        print('tcpdump not available; skipping pcap', file=sys.stderr)
        return None


def start_rosbag(topics, out_dir):
    try:
        return subprocess.Popen(['ros2', 'bag', 'record', '-o', out_dir, *topics],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                start_new_session=True)
    except FileNotFoundError:
        print('ros2 not available; skipping rosbag', file=sys.stderr)
        return None


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)


TOPICS = ('/plc/reference', '/robot/reference', '/robot/measurement', '/ids/status',
          '/ids_thesis/status', '/ids/alarm', '/attack/truth', '/joint_states')


def run(args):
    os.makedirs(args.out, exist_ok=True)
    pcap = start_pcap(args.interface, os.path.join(args.out, 'cell.pcap')) if args.pcap else None
    bag = start_rosbag(TOPICS, os.path.join(args.out, 'bag')) if args.rosbag else None
    campaign = None
    try:
        if args.command:
            campaign = subprocess.Popen(args.command)
            campaign.wait()
        else:
            print('range capturing; Ctrl-C to stop', flush=True)
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if campaign is not None and campaign.poll() is None:
            campaign.terminate()
        stop(bag)
        stop(pcap)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Range orchestrator: packet capture and rosbag recording around a run.')
    parser.add_argument('--out', default='/ws/data/cell/range')
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--pcap', action='store_true')
    parser.add_argument('--rosbag', action='store_true')
    parser.add_argument('command', nargs=argparse.REMAINDER,
                        help='optional run command to wrap, after --')
    args = parser.parse_args(argv)
    return run(args)


if __name__ == '__main__':
    sys.exit(main())