import numpy as np
import yaml

from ids_attacks.library import AttackSchedule, make_attack
from ids_experiments.reports import NodeReport, security_context
from ids_experiments.rogue import (blind_position, HOME, JOINT_NAMES, offset_position,
                                   spoofed_state, stamp_of)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_security_context_reads_the_sros2_environment():
    env = {'ROS_SECURITY_ENABLE': 'true', 'ROS_SECURITY_STRATEGY': 'Enforce',
           'ROS_SECURITY_ENCLAVE_OVERRIDE': '/attacker/implant', 'ROS_DOMAIN_ID': '10'}
    assert security_context(env) == {'security_enabled': True, 'strategy': 'Enforce',
                                     'enclave': '/attacker/implant', 'domain': 10}
    off = security_context({'ROS_SECURITY_ENABLE': 'false',
                            'ROS_SECURITY_ENCLAVE_OVERRIDE': '/x'})
    assert off == {'security_enabled': False, 'strategy': '', 'enclave': '', 'domain': 0}


def test_report_counts_matches_and_writes_periodically(tmp_path):
    clock = Clock()
    path = tmp_path / 'r' / 'rogue.yaml'
    report = NodeReport(str(path), 'rogue', {'scenario': 's'}, period=1.0, clock=clock,
                        env={'ROS_SECURITY_ENABLE': 'false'})
    report.count('injected_blind')
    report.count('injected_blind', 2)
    report.observe('target_subscribers', 0)
    report.observe('target_subscribers', 2)
    report.observe('target_subscribers', 1)
    report.maybe_write()
    first = yaml.safe_load(path.read_text())
    assert first['counts'] == {'injected_blind': 3} and first['final'] is False
    assert first['matched']['target_subscribers'] == {'max': 2, 'last': 1}
    assert first['scenario'] == 's' and first['security_enabled'] is False
    report.count('injected_blind')
    clock.t = 0.5
    report.maybe_write()
    assert yaml.safe_load(path.read_text())['counts']['injected_blind'] == 3
    clock.t = 1.5
    report.write(final=True)
    last = yaml.safe_load(path.read_text())
    assert last['counts']['injected_blind'] == 4 and last['final'] and last['elapsed_s'] == 1.5
    assert NodeReport('', 'x').write() is None


def test_blind_and_spoofed_positions_add_the_attack_offsets():
    step = make_attack(1, 'step', (3,), 0.02, 120.0, 10.0)
    assert np.allclose(blind_position(HOME, [step]) - np.array(HOME),
                       [0, 0, 0, 0.02, 0, 0, 0])
    assert np.allclose(blind_position(HOME, []), HOME)
    coordinated = make_attack(2, 'coordinated', (1, 3), None, 0.0, 1.0,
                              magnitudes=(0.1, -0.2))
    assert np.allclose(offset_position(np.zeros(7), coordinated), [0, 0.1, 0, -0.2, 0, 0, 0])
    schedule = AttackSchedule([step])
    assert spoofed_state(schedule, 119.9, np.zeros(7)) is None
    assert np.allclose(spoofed_state(schedule, 125.0, np.ones(7)),
                       [1, 1, 1, 1.02, 1, 1, 1])
    assert JOINT_NAMES[0] == 'joint_a1' and len(JOINT_NAMES) == 7


def test_stamp_of():
    class Stamp:
        sec, nanosec = 12, 500_000_000

    class Header:
        stamp = Stamp()

    class Msg:
        header = Header()

    assert stamp_of(Msg()) == 12.5