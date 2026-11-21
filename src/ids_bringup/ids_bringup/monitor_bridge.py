from dataclasses import dataclass
import os
import sys

import yaml

SIDES = ('monitor', 'cell')
ROUTE_KEYS = {'topic', 'type', 'to', 'depth', 'latched'}


@dataclass(frozen=True)
class Route:
    topic: str
    type: str
    to: str
    depth: int = 100
    latched: bool = False

    @property
    def source(self):
        return 'cell' if self.to == 'monitor' else 'monitor'


def parse_routes(data):
    if not isinstance(data, dict):
        raise ValueError('bridge config must be a mapping')
    unknown = set(data) - {'monitor_domain', 'routes'}
    if unknown:
        raise ValueError(f'unknown keys in bridge config: {sorted(unknown)}')
    domain = data.get('monitor_domain', 20)
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
        raise ValueError('monitor_domain must be an integer from 0 to 232')
    routes = []
    seen = set()
    for i, r in enumerate(data.get('routes') or []):
        if not isinstance(r, dict):
            raise ValueError(f'route {i} must be a mapping')
        unknown = set(r) - ROUTE_KEYS
        if unknown:
            raise ValueError(f'route {i}: unknown keys {sorted(unknown)}')
        topic, kind, to = r.get('topic'), r.get('type'), r.get('to')
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise ValueError(f'route {i}: topic must be an absolute ROS name')
        if not isinstance(kind, str) or kind.count('/') != 2:
            raise ValueError(f'route {i}: type must look like package/msg/Name')
        if to not in SIDES:
            raise ValueError(f'route {i}: to must be one of {SIDES}')
        depth = r.get('depth', 1 if r.get('latched') else 100)
        if isinstance(depth, bool) or not isinstance(depth, int) or depth < 1:
            raise ValueError(f'route {i}: depth must be a positive integer')
        latched = r.get('latched', False)
        if not isinstance(latched, bool):
            raise ValueError(f'route {i}: latched must be true or false')
        if topic in seen:
            raise ValueError(f'route {i}: {topic} is routed twice')
        seen.add(topic)
        routes.append(Route(topic, kind, to, depth, latched))
    if not routes:
        raise ValueError('the bridge needs at least one route')
    return domain, tuple(routes)


def load_routes(path):
    with open(path) as f:
        return parse_routes(yaml.safe_load(f))


def default_config():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ids_bringup'), 'config',
                        'monitor_bridge.yaml')


class Counter:

    def __init__(self, routes):
        self.counts = {r.topic: 0 for r in routes}

    def add(self, topic):
        self.counts[topic] += 1

    def line(self):
        return ', '.join(f'{t} {n}' for t, n in self.counts.items())


def _qos(route):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=route.depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=(DurabilityPolicy.TRANSIENT_LOCAL if route.latched
                                  else DurabilityPolicy.VOLATILE))


def build(nodes, routes, counter):
    from rosidl_runtime_py.utilities import get_message
    keep = []
    for route in routes:
        kind = get_message(route.type)
        publisher = nodes[route.to].create_publisher(kind, route.topic, _qos(route))

        def forward(data, publisher=publisher, topic=route.topic):
            publisher.publish(data)
            counter.add(topic)

        keep.append(nodes[route.source].create_subscription(kind, route.topic, forward,
                                                            _qos(route), raw=True))
        keep.append(publisher)
    return keep


def main(argv=None):
    from rclpy.context import Context
    from rclpy.node import Node

    from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag

    argv = sys.argv if argv is None else argv
    init(args=argv)
    cell = Node('monitor_bridge')
    cell.declare_parameter('config_file', '')
    cell.declare_parameter('monitor_domain', -1)
    path = cell.get_parameter('config_file').value or default_config()
    domain, routes = load_routes(path)
    override = int(cell.get_parameter('monitor_domain').value)
    domain = override if override >= 0 else domain
    context = Context()
    init(args=[], context=context, domain_id=domain)
    monitor = Node('monitor_bridge', context=context)
    counter = Counter(routes)
    keep = build({'cell': cell, 'monitor': monitor}, routes, counter)
    for r in routes:
        cell.get_logger().info(f'{r.topic} ({r.type}): {r.source} -> {r.to}'
                               f'{" (latched)" if r.latched else ""}')
    cell.get_logger().info(f'bridging the cell domain and the monitor domain {domain}')
    cell.create_timer(30.0, lambda: cell.get_logger().info(f'forwarded: {counter.line()}'))
    spin_until_shutdown(cell, monitor, stop=StopFlag())
    del keep
    return 0