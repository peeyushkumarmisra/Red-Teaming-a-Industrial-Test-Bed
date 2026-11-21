import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ids_security.policy import (allowed, dds_topic, default_policy_path, load_policy,  # noqa
                                 parse_policy, rules)

CELL, MONITOR, PROBE = 10, 20, 30

NEEDS = [
    ('/cell/plc', CELL, 'publish', '/plc/reference'),
    ('/cell/plc', CELL, 'publish', '/gripper/attach'),
    ('/cell/plc', CELL, 'publish', '/gripper/detach'),
    ('/cell/plc', CELL, 'subscribe', '/joint_states'),
    ('/cell/plc', CELL, 'subscribe', '/robot_description'),
    ('/cell/plc', CELL, 'subscribe', '/gripper/state'),
    ('/cell/plc', CELL, 'subscribe', '/clock'),
    ('/cell/plc', CELL, 'publish', 'rq/world/cell/set_poseRequest'),
    ('/cell/plc', CELL, 'subscribe', 'rr/world/cell/set_poseReply'),
    ('/cell/plc', MONITOR, 'publish', '/plc/reference'),
    ('/cell/plc', PROBE, 'publish', '/probe/ping'),
    ('/cell/plc', PROBE, 'subscribe', '/probe/pong'),
    ('/cell/robot', CELL, 'subscribe', '/plc/reference'),
    ('/cell/robot', CELL, 'subscribe', '/robot/reference'),
    ('/cell/robot', CELL, 'publish', '/robot/measurement'),
    ('/cell/robot', CELL, 'publish', '/robot/measurement_raw'),
    ('/cell/robot', CELL, 'publish', '/clock'),
    ('/cell/robot', CELL, 'publish', '/joint_states'),
    ('/cell/robot', CELL, 'publish', '/ids/alarm'),
    ('/cell/robot', CELL, 'publish', '/ids/status'),
    ('/cell/robot', CELL, 'subscribe', '/attack/truth'),
    ('/cell/robot', CELL, 'publish', '/events/write_split'),
    ('/cell/robot', MONITOR, 'publish', '/robot/measurement'),
    ('/cell/robot', MONITOR, 'publish', '/robot_description'),
    ('/cell/robot', MONITOR, 'subscribe', '/ids/alarm'),
    ('/cell/robot', MONITOR, 'subscribe', '/ids_thesis/status'),
    ('/cell/robot', PROBE, 'subscribe', '/probe/ping'),
    ('/cell/robot', PROBE, 'publish', '/probe/pong'),
    ('/monitor/ids', MONITOR, 'subscribe', '/plc/reference'),
    ('/monitor/ids', MONITOR, 'subscribe', '/robot/measurement'),
    ('/monitor/ids', MONITOR, 'subscribe', '/robot_description'),
    ('/monitor/ids', MONITOR, 'publish', '/ids/status'),
    ('/monitor/ids', MONITOR, 'publish', '/ids/alarm'),
    ('/monitor/ids', MONITOR, 'publish', '/ids_thesis/alarm'),
    ('/monitor/ids', MONITOR, 'subscribe', 'rq/ids/ids_node/resetRequest'),
    ('/attacker/implant', CELL, 'subscribe', '/plc/reference'),
    ('/attacker/implant', CELL, 'publish', '/robot/reference'),
    ('/attacker/implant', CELL, 'publish', '/attack/truth'),
    ('/attacker/implant', CELL, 'subscribe', '/robot/measurement_raw'),
    ('/attacker/implant', CELL, 'publish', '/robot/measurement'),
    ('/attacker/implant', CELL, 'publish', '/joint_states'),
]

REFUSED = [
    ('/cell/robot', CELL, 'publish', '/plc/reference'),
    ('/cell/robot', CELL, 'publish', '/robot/reference'),
    ('/cell/robot', CELL, 'publish', '/attack/truth'),
    ('/cell/robot', MONITOR, 'publish', '/plc/reference'),
    ('/cell/robot', MONITOR, 'publish', '/ids/alarm'),
    ('/cell/plc', CELL, 'publish', '/robot/reference'),
    ('/cell/plc', CELL, 'publish', '/robot/measurement'),
    ('/cell/plc', MONITOR, 'publish', '/robot/measurement'),
    ('/cell/plc', MONITOR, 'subscribe', '/ids/status'),
    ('/monitor/ids', MONITOR, 'publish', '/plc/reference'),
    ('/monitor/ids', MONITOR, 'publish', '/robot/measurement'),
    ('/monitor/ids', CELL, 'subscribe', '/plc/reference'),
    ('/attacker/implant', MONITOR, 'subscribe', '/plc/reference'),
    ('/attacker/implant', CELL, 'publish', '/plc/reference'),
    ('/attacker/implant', CELL, 'publish', '/ids/alarm'),
]


@pytest.fixture(scope='module')
def policy():
    return load_policy(default_policy_path())


def test_shipped_policy_loads_with_three_domains(policy):
    assert policy.domains == {'cell': 10, 'monitor': 20, 'probe': 30}
    legit = {e.path for e in policy.in_store('keystore')}
    assert legit == {'/cell/robot', '/cell/plc', '/monitor/ids'}
    assert {e.path for e in policy.in_store('loot')} == {'/attacker/implant', '/attacker/forged'}
    assert policy.enclave('/attacker/forged').ca == 'forged'


@pytest.mark.parametrize('enclave,domain,action,topic', NEEDS)
def test_every_node_gets_what_it_uses(policy, enclave, domain, action, topic):
    assert allowed(policy, policy.enclave(enclave), domain, action, topic)


@pytest.mark.parametrize('enclave,domain,action,topic', REFUSED)
def test_security_relevant_topics_are_refused(policy, enclave, domain, action, topic):
    assert not allowed(policy, policy.enclave(enclave), domain, action, topic)


def test_rule_order_lets_the_robot_read_what_it_may_not_write(policy):
    robot = policy.enclave('/cell/robot')
    kinds = [(r.kind, r.domains, bool(r.publish), bool(r.subscribe))
             for r in rules(policy, robot)]
    assert kinds[:3] == [('allow_rule', (10,), False, True), ('deny_rule', (10,), True, False),
                         ('allow_rule', (10,), True, False)]
    for topic in ('/plc/reference', '/robot/reference', '/attack/truth'):
        assert allowed(policy, robot, CELL, 'subscribe', topic)
        assert not allowed(policy, robot, CELL, 'publish', topic)


def test_a_deny_rule_first_would_block_even_reading_the_topic():
    p = parse_policy({'domains': {'cell': 10}, 'enclaves': {'/a/b': {'domains': {'cell': {
        'deny_publish': ['/x'], 'publish': ['*']}}}}})
    e = p.enclave('/a/b')
    assert not allowed(p, e, 10, 'subscribe', '/x') and not allowed(p, e, 10, 'publish', '/x')
    assert allowed(p, e, 10, 'publish', '/y')


def test_dds_names():
    assert dds_topic('/plc/reference') == 'rt/plc/reference'
    assert dds_topic('ros_discovery_info') == 'ros_discovery_info'
    assert dds_topic('*') == '*'


def base():
    return {'domains': {'cell': 10},
            'enclaves': {'/a/b': {'domains': {'cell': {'publish': ['/x']}}}}}


@pytest.mark.parametrize('change,message', [
    (lambda d: d['enclaves'].update({'/a/b/c': {'domains': {'cell': {}}}}), 'nested'),
    (lambda d: d['enclaves'].update({'bad': {'domains': {'cell': {}}}}), 'absolute'),
    (lambda d: d['enclaves'].update({'/a/9x': {'domains': {'cell': {}}}}), 'segment'),
    (lambda d: d['enclaves']['/a/b']['domains'].update({'other': {}}), 'unknown domain'),
    (lambda d: d['enclaves']['/a/b'].update({'ca': 'forged'}), 'loot store'),
    (lambda d: d['enclaves']['/a/b'].update({'domains': {}}), 'at least one domain'),
    (lambda d: d['domains'].update({'twin': 10}), 'unique'),
    (lambda d: d['domains'].update({'big': 999}), '0 to 232'),
    (lambda d: d['enclaves']['/a/b']['domains']['cell'].update({'publish': '/x'}), 'list'),
    (lambda d: d.update({'extra': 1}), 'unknown keys'),
    (lambda d: d.update({'authority': {'validity_days': 0}}), 'validity_days'),
])
def test_policy_errors(change, message):
    data = base()
    change(data)
    with pytest.raises(ValueError, match=message):
        parse_policy(data)


def test_minimal_policy_round_trip(tmp_path):
    path = tmp_path / 'p.yaml'
    path.write_text(yaml.safe_dump(base()))
    p = load_policy(str(path))
    e = p.enclave('/a/b')
    assert allowed(p, e, 10, 'publish', '/x') and not allowed(p, e, 10, 'publish', '/y')
    assert not allowed(p, e, 11, 'publish', '/x')
    with pytest.raises(ValueError):
        allowed(p, e, 10, 'relay', '/x')
    with pytest.raises(KeyError):
        p.enclave('/missing')