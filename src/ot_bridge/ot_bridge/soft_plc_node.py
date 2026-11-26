import argparse
import sys

import numpy as np
import yaml

from ot_bridge.soft_plc import build_program, PlcCore, run_server

HOME = [0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0]


def load_stations(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    start = np.array(data.get('start', HOME), dtype=float)
    stations = {s: {k: np.array(data['stations'][s][k], dtype=float)
                    for k in ('hover', 'grab', 'drop')} for s in ('a', 'b')}
    places = data.get('places')
    places = tuple(tuple(p) for p in places) if places else None
    return start, stations, places, float(data.get('payload_mass', 5.0))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Python soft-PLC: runs the discrete sequencer and serves the Modbus map '
                    '(a comment-compatible stand-in for the OpenPLC ST program).')
    parser.add_argument('--teach', required=True, help='teach-point YAML from plc/teach_points.py')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=502)
    parser.add_argument('--cycles', type=int, default=0)
    parser.add_argument('--scan-period', type=float, default=0.01)
    args = parser.parse_args(argv)
    start, stations, places, mass = load_stations(args.teach)
    program = build_program(start, stations, payload_mass=mass, places=places)
    core = PlcCore(start, program, passes=args.cycles, scan_period=args.scan_period)
    print(f'soft PLC serving the Modbus map on {args.host}:{args.port}, '
          f'{len(program)} program steps', flush=True)
    run_server(core, host=args.host, port=args.port, scan_period=args.scan_period)
    return 0


if __name__ == '__main__':
    sys.exit(main())