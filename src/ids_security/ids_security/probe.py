import argparse
import array
import json
import sys
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag
from ids_security.latency import pack, summarise, unpack

PING_TOPIC = '/probe/ping'
PONG_TOPIC = '/probe/pong'


def qos():
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)


class Pong(Node):

    def __init__(self):
        super().__init__('probe_pong')
        self._pub = self.create_publisher(UInt8MultiArray, PONG_TOPIC, qos())
        self.create_subscription(UInt8MultiArray, PING_TOPIC, self._echo, qos(), raw=True)
        self.echoed = 0
        self.get_logger().info('echoing /probe/ping on /probe/pong')

    def _echo(self, data):
        self._pub.publish(data)
        self.echoed += 1


class Ping(Node):

    def __init__(self, count, rate, size, warmup, timeout, stop):
        super().__init__('probe_ping')
        self._count, self._size, self._warmup = count, size, warmup
        self._timeout, self._stop = timeout, stop
        self._pub = self.create_publisher(UInt8MultiArray, PING_TOPIC, qos())
        self._sub = self.create_subscription(UInt8MultiArray, PONG_TOPIC, self._on_pong, qos())
        self._seq = 0
        self._rtt = []
        self._started = time.monotonic()
        self._measure_from = None
        self._cpu0 = self._wall0 = None
        self._last_sent = None
        self.result = None
        self.create_timer(1.0 / rate, self._tick)

    def _matched(self):
        return self._pub.get_subscription_count() > 0 and self._sub.get_publisher_count() > 0

    def _tick(self):
        now = time.monotonic()
        if self._seq == 0 and not self._matched():
            if now - self._started > self._timeout:
                self._finish('no pong matched within the timeout')
            return
        if self._seq < self._warmup + self._count:
            if self._seq == self._warmup:
                self._measure_from = self._seq
                self._cpu0, self._wall0 = time.process_time(), now
            msg = UInt8MultiArray()
            msg.data = array.array('B', pack(self._seq, time.monotonic_ns(), self._size))
            self._pub.publish(msg)
            self._seq += 1
            self._last_sent = now
        elif now - self._last_sent > 1.0:
            self._finish(None)

    def _on_pong(self, msg):
        seq, sent = unpack(msg.data)
        if self._measure_from is not None and seq >= self._measure_from:
            self._rtt.append(time.monotonic_ns() - sent)

    def _finish(self, error):
        if self.result is not None:
            return
        cpu = time.process_time() - self._cpu0 if self._cpu0 is not None else float('nan')
        wall = time.monotonic() - self._wall0 if self._wall0 is not None else float('nan')
        self.result = summarise(self._rtt, max(0, self._seq - self._warmup), cpu, wall)
        self.result['error'] = error
        self._stop.set()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Round-trip latency probe')
    parser.add_argument('role', choices=('ping', 'pong'))
    parser.add_argument('--count', type=int, default=3000)
    parser.add_argument('--rate', type=float, default=100.0)
    parser.add_argument('--size', type=int, default=256)
    parser.add_argument('--warmup', type=int, default=100)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--out', default='')
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    init()
    stop = StopFlag()
    if args.role == 'pong':
        spin_until_shutdown(Pong(), stop=stop)
        return 0
    node = Ping(args.count, args.rate, args.size, args.warmup, args.timeout, stop)
    spin_until_shutdown(node, stop=stop)
    result = node.result or {'error': 'interrupted'}
    text = json.dumps(result, indent=1)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text)
    print(text)
    return 0 if not result.get('error') else 1