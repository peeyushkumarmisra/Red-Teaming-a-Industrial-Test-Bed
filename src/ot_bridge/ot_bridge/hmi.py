import argparse
import html
import http.server
import json
import threading

from ot_bridge.modbus_io import read_command, read_status, TcpStore
from ot_bridge.modbus_map import COIL_IDS_ALARM, COIL_RUN, COIL_SAFE_STOP

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>Cell HMI</title><meta http-equiv="refresh" content="1">
<style>body{{font-family:sans-serif;margin:2rem}}.alarm{{color:#b00}}
.ok{{color:#070}}td{{padding:.2rem .8rem}}</style></head><body>
<h1>Robot cell</h1><p class="{cls}">IDS alarm: <b>{alarm}</b></p>
<table><tr><td>program running</td><td>{run}</td></tr>
<tr><td>safe stop</td><td>{safe}</td></tr><tr><td>step</td><td>{step}</td></tr>
<tr><td>payload</td><td>{context}</td></tr><tr><td>gripper</td><td>{gripper}</td></tr>
<tr><td>reference seq</td><td>{ref_seq}</td></tr><tr><td>robot halted</td><td>{halted}</td></tr>
</table></body></html>"""


class CellState:

    def __init__(self):
        self.lock = threading.Lock()
        self.data = {'alarm': False, 'run': False, 'safe': False, 'step': 0, 'context': 0,
                     'gripper': 0, 'ref_seq': 0, 'halted': False, 'connected': False}

    def update(self, store):
        try:
            command = read_command(store)
            status = read_status(store)
            new = {'alarm': store.read_coil(COIL_IDS_ALARM), 'run': store.read_coil(COIL_RUN),
                   'safe': store.read_coil(COIL_SAFE_STOP), 'step': int(command.step),
                   'context': int(command.context), 'gripper': int(command.gripper),
                   'ref_seq': status['reference_seq'], 'halted': status['halted'],
                   'connected': True}
        except (IOError, OSError):
            new = dict(self.data)
            new['connected'] = False
        with self.lock:
            self.data = new

    def snapshot(self):
        with self.lock:
            return dict(self.data)


def make_handler(state):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            data = state.snapshot()
            if self.path.startswith('/state'):
                body = json.dumps(data).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
            else:
                page = PAGE.format(
                    cls='alarm' if data['alarm'] else 'ok',
                    alarm='RAISED' if data['alarm'] else 'clear',
                    run=data['run'], safe=data['safe'], step=html.escape(str(data['step'])),
                    context=data['context'], gripper=data['gripper'], ref_seq=data['ref_seq'],
                    halted=data['halted'])
                body = page.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return Handler


def poll_loop(state, store, period, stop):
    while not stop.is_set():
        state.update(store)
        stop.wait(period)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read-only web HMI for the robot cell.')
    parser.add_argument('--plc-host', default='10.203.10.20')
    parser.add_argument('--plc-port', type=int, default=502)
    parser.add_argument('--listen-port', type=int, default=8081)
    parser.add_argument('--period', type=float, default=0.5)
    args = parser.parse_args(argv)
    state = CellState()
    store = TcpStore(args.plc_host, args.plc_port)
    store.connect()
    stop = threading.Event()
    threading.Thread(target=poll_loop, args=(state, store, args.period, stop), daemon=True).start()
    server = http.server.ThreadingHTTPServer(('0.0.0.0', args.listen_port), make_handler(state))
    print(f'HMI on :{args.listen_port}, polling the PLC at {args.plc_host}:{args.plc_port}',
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
    return 0


if __name__ == '__main__':
    main()