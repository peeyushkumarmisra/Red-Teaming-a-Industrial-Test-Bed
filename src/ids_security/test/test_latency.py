import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ids_security import overhead  # noqa: E402
from ids_security.latency import pack, report_lines, summarise, unpack  # noqa: E402


def test_pack_round_trip_and_padding():
    data = pack(7, 123456789, 256)
    assert len(data) == 256 and unpack(data) == (7, 123456789)
    assert len(pack(1, 2, 4)) == 16


def test_summary_percentiles_loss_and_cpu():
    rtt = [1_000_000] * 98 + [5_000_000, 9_000_000]
    s = summarise(rtt, sent=104, cpu_seconds=0.5, wall_seconds=10.0)
    assert s['received'] == 100 and s['lost'] == 4
    assert s['rtt_median_ms'] == pytest.approx(1.0)
    assert s['rtt_max_ms'] == pytest.approx(9.0)
    assert s['cpu_percent'] == pytest.approx(5.0)
    empty = summarise([], sent=10)
    assert empty['lost'] == 10 and math.isnan(empty['rtt_median_ms'])
    assert math.isnan(empty['cpu_percent'])


def test_report_has_difference_and_ratio():
    a = summarise([2_000_000] * 10, 10, 1.0, 10.0)
    b = summarise([1_000_000] * 10, 10, 0.5, 10.0)
    text = '\n'.join(report_lines(a, b, 256, 100.0))
    assert '| round trip median (ms) | 2.000 | 1.000 | +1.000 | 2.00 |' in text


def test_mode_env_switches_domain_and_security():
    base = {'ROS_DOMAIN_ID': '10', 'ROS_SECURITY_ENABLE': 'true', 'X': '1'}
    assert overhead.mode_env('sros2', base) == {'ROS_DOMAIN_ID': '30',
                                                'ROS_SECURITY_ENABLE': 'true', 'X': '1'}
    assert overhead.mode_env('plain', base)['ROS_SECURITY_ENABLE'] == 'false'
    assert overhead.mode_env('plain', base)['ROS_DOMAIN_ID'] == '31'
    with pytest.raises(ValueError):
        overhead.mode_env('fast', base)


def test_ping_runs_both_modes_and_writes_the_report(tmp_path):
    calls = []

    def run(command, env, check):
        calls.append((command, env['ROS_DOMAIN_ID'], env['ROS_SECURITY_ENABLE']))
        out = command[command.index('--out') + 1]
        rtt = [2_000_000] * 50 if env['ROS_SECURITY_ENABLE'] == 'true' else [1_000_000] * 50
        with open(out, 'w') as f:
            json.dump(summarise(rtt, 50, 0.1, 1.0) | {'error': None}, f)

    assert overhead.run_ping(str(tmp_path), 50, 100.0, 256, run=run) == 0
    assert [c[1:] for c in calls] == [('30', 'true'), ('31', 'false')]
    assert calls[0][0][:5] == ['ros2', 'run', 'ids_security', 'probe', 'ping']
    assert 'SROS2 overhead' in (tmp_path / 'report.md').read_text()
    assert (tmp_path / 'overhead.yaml').exists()


def test_ping_stops_on_a_probe_error(tmp_path):
    def run(command, env, check):
        with open(command[command.index('--out') + 1], 'w') as f:
            json.dump({'error': 'no pong matched within the timeout'}, f)

    with pytest.raises(SystemExit, match='no pong'):
        overhead.run_ping(str(tmp_path), 10, 100.0, 256, run=run)


def test_pong_starts_one_echo_per_mode(monkeypatch):
    started, signals = [], []

    class Proc:
        def __init__(self, command, env, start_new_session):
            started.append((command, env['ROS_DOMAIN_ID']))
            self.pid = 1000 + len(started)

        def poll(self):
            return None

        def wait(self, timeout=None):
            return 0

    monkeypatch.setattr(overhead.os, 'killpg', lambda pid, sig: signals.append(pid))
    assert overhead.run_pong(popen=Proc, wait_forever=lambda: None) == 0
    assert [s[1] for s in started] == ['30', '31']
    assert signals == [1001, 1002]