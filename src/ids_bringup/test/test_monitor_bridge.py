import os

import pytest

from ids_bringup.monitor_bridge import Counter, load_routes, parse_routes, Route

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config', 'monitor_bridge.yaml')


def test_shipped_routes_carry_measurements_out_and_alarms_back():
    domain, routes = load_routes(CONFIG)
    assert domain == 20
    by_topic = {r.topic: r for r in routes}
    assert by_topic['/robot/measurement'] == Route('/robot/measurement',
                                                   'ids_interfaces/msg/JointMeasurement',
                                                   'monitor', 100, False)
    assert by_topic['/robot_description'].latched and by_topic['/robot_description'].depth == 1
    for topic in ('/ids/alarm', '/ids_thesis/alarm'):
        assert by_topic[topic].to == 'cell' and by_topic[topic].latched
        assert by_topic[topic].source == 'monitor'
    assert '/plc/reference' not in by_topic
    assert all(not r.topic.startswith('/attack') for r in routes)


def route(**kw):
    return dict({'topic': '/a', 'type': 'std_msgs/msg/Bool', 'to': 'cell'}, **kw)


@pytest.mark.parametrize('data,message', [
    ([], 'mapping'),
    ({'routes': []}, 'at least one route'),
    ({'monitor_domain': 300, 'routes': [route()]}, 'monitor_domain'),
    ({'routes': [route(topic='a')]}, 'absolute'),
    ({'routes': [route(type='Bool')]}, 'package/msg/Name'),
    ({'routes': [route(to='both')]}, 'to must be'),
    ({'routes': [route(depth=0)]}, 'depth'),
    ({'routes': [route(latched='yes')]}, 'latched'),
    ({'routes': [route(), route(to='monitor')]}, 'routed twice'),
    ({'routes': [route(extra=1)]}, 'unknown keys'),
    ({'routes': ['x']}, 'mapping'),
    ({'other': 1, 'routes': [route()]}, 'unknown keys'),
])
def test_invalid_configs(data, message):
    with pytest.raises(ValueError, match=message):
        parse_routes(data)


def test_counter_line():
    c = Counter([Route('/a', 'p/msg/A', 'cell'), Route('/b', 'p/msg/B', 'monitor')])
    c.add('/a')
    c.add('/a')
    assert c.line() == '/a 2, /b 0'