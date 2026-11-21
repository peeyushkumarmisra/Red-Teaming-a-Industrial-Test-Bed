import argparse
from dataclasses import dataclass, field
import os
import signal
import subprocess
import sys
import threading
import time

from ids_bringup.cell import (CELL_DIR, read_run, report_dir, ROLES, write_heartbeat,
                              write_yaml)

ENCLAVES = {'stolen': '/attacker/implant', 'forged': '/attacker/forged'}
EXECUTABLES = {'mitm': 'mitm', 'rogue': 'rogue', 'spoof': 'joint_states_spoof'}


@dataclass
class Process:
    name: str
    command: list
    env: dict = field(default_factory=dict)
    unset: tuple = ()


def credentials_env(credentials):
    if credentials == 'none':
        return {'ROS_SECURITY_ENABLE': 'false'}, ('ROS_SECURITY_ENCLAVE_OVERRIDE',)
    if credentials in ENCLAVES:
        return {'ROS_SECURITY_ENABLE': 'true',
                'ROS_SECURITY_ENCLAVE_OVERRIDE': ENCLAVES[credentials]}, ()
    raise ValueError(f'unknown credentials {credentials}')


def _ros_run(package, executable, params):
    command = ['ros2', 'run', package, executable, '--ros-args']
    for key, value in params.items():
        if isinstance(value, bool):
            value = str(value).lower()
        command += ['-p', f'{key}:={value}']
    return command


def plc_processes(run, reports):
    plc = run['roles']['plc']
    return [Process('plc_planner', _ros_run('ids_experiments', 'plc_planner', {
        'use_sim_time': True, 'cycles': int(plc['cycles']),
        'payload_mass': float(plc['payload_mass']),
        'mirror_domain': int(plc['mirror_domain'])}))]


def attacker_processes(run, reports):
    spec = run['roles']['attacker']
    role = spec['role']
    if role == 'none':
        return []
    if role not in EXECUTABLES:
        raise ValueError(f'unknown attacker role {role}')
    scenario_file = os.path.join(reports, 'attacker_scenario.yaml')
    write_yaml(scenario_file, run['scenario'])
    params = {'scenario_file': scenario_file,
              'report_file': os.path.join(reports, f'{role}.yaml')}
    if role == 'mitm':
        params['use_sim_time'] = True
    env, unset = credentials_env(spec['credentials'])
    return [Process(role, _ros_run('ids_experiments', EXECUTABLES[role], params), env, unset)]


PLANS = {'plc': plc_processes, 'attacker': attacker_processes}


def process_env(process, base=os.environ):
    env = {k: v for k, v in base.items() if k not in process.unset}
    env.update(process.env)
    return env


class Agent:

    def __init__(self, role, cell_dir=CELL_DIR, popen=subprocess.Popen, killpg=os.killpg,
                 clock=time.time, sleep=time.sleep, log=print, stop_timeout=10.0,
                 base_env=os.environ):
        if role not in ROLES:
            raise ValueError(f'role must be one of {ROLES}')
        self.role = role
        self.cell_dir = cell_dir
        self._popen, self._killpg, self._clock = popen, killpg, clock
        self._sleep, self._log, self._stop_timeout = sleep, log, stop_timeout
        self._base_env = base_env
        self.current = None
        self.procs = []
        self._exited = set()

    def desired(self):
        run = read_run(self.cell_dir)
        if run is None or run['state'] != 'running':
            return None, None
        return run['run_id'], run

    def _record(self, run_id, processes, event):
        path = os.path.join(report_dir(self.cell_dir, run_id), f'{self.role}_agent.yaml')
        write_yaml(path, {'role': self.role, 'run_id': run_id, 'event': event,
                          'time': self._clock(),
                          'processes': [{'name': p.name, 'command': p.command, 'env': p.env,
                                         'unset': list(p.unset),
                                         'returncode': getattr(proc, 'returncode', None)}
                                        for p, proc in processes]})

    def start(self, run_id, run):
        reports = report_dir(self.cell_dir, run_id)
        os.makedirs(reports, exist_ok=True)
        try:
            processes = PLANS[self.role](run, reports)
        except (KeyError, TypeError, ValueError) as e:
            self._log(f'[{self.role}] run {run_id}: cannot plan processes: {e}')
            processes = []
        self.procs = []
        for p in processes:
            proc = self._popen(p.command, env=process_env(p, self._base_env),
                               start_new_session=True)
            self.procs.append((p, proc))
            extra = ' '.join(f'{k}={v}' for k, v in p.env.items())
            self._log(f'[{self.role}] run {run_id}: started {p.name} (pid {proc.pid}) {extra}')
        if not processes:
            self._log(f'[{self.role}] run {run_id}: nothing to run')
        self.current = run_id
        self._exited = set()
        self._record(run_id, self.procs, 'started')

    def _signal(self, proc, sig):
        try:
            self._killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def stop(self):
        if self.current is None:
            return
        for _, proc in self.procs:
            if proc.poll() is None:
                self._signal(proc, signal.SIGINT)
        deadline = self._clock() + self._stop_timeout
        for sig in (signal.SIGTERM, signal.SIGKILL):
            while any(proc.poll() is None for _, proc in self.procs) \
                    and self._clock() < deadline:
                self._sleep(0.1)
            for _, proc in self.procs:
                if proc.poll() is None:
                    self._signal(proc, sig)
            deadline = self._clock() + 2.0
        for p, proc in self.procs:
            self._log(f'[{self.role}] run {self.current}: {p.name} stopped '
                      f'(exit {proc.poll()})')
        self._record(self.current, self.procs, 'stopped')
        self.current = None
        self.procs = []

    def step(self):
        run_id, run = self.desired()
        if run_id != self.current:
            self.stop()
            if run_id is not None:
                self.start(run_id, run)
        for p, proc in self.procs:
            code = proc.poll()
            if code is not None and p.name not in self._exited:
                self._exited.add(p.name)
                self._log(f'[{self.role}] run {self.current}: {p.name} exited with {code}')
        write_heartbeat(self.cell_dir, self.role, self.current, os.getpid(), self._clock())

    def run(self, stop, period=0.5):
        self._log(f'[{self.role}] agent watching {self.cell_dir}/run.yaml')
        try:
            while not stop.is_set():
                self.step()
                stop.wait(period)
        finally:
            self.stop()
            self._log(f'[{self.role}] agent stopped')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Start and stop this container\'s nodes for each run of run_campaign.')
    parser.add_argument('role', choices=ROLES)
    parser.add_argument('--cell-dir', default=CELL_DIR)
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    stop = threading.Event()
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.set())
    Agent(args.role, args.cell_dir, log=lambda text: print(text, flush=True)).run(stop)
    return 0