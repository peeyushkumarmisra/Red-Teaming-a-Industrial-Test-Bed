from rclpy.node import Node
from sensor_msgs.msg import JointState

from ids_attacks.library import AttackSchedule
from ids_experiments.reports import NodeReport
from ids_experiments.rogue import blind_position, HOME, JOINT_NAMES, MARK, spoofed_state, stamp_of
from ids_experiments.scenario import load_scenario
from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag
from ids_interfaces.msg import JointReference


class JointStateSpoofer(Node):

    def __init__(self):
        super().__init__('joint_states_spoof')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('listen_topic', '/plc/reference')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('blind', False)
        self.declare_parameter('home', list(HOME))
        self.declare_parameter('report_file', '')
        path = self.get_parameter('scenario_file').value
        scenario = load_scenario(path) if path else None
        attacks = scenario.attacks if scenario else ()
        self._schedule = AttackSchedule(attacks)
        self._blind_q = blind_position(self.get_parameter('home').value, attacks)
        self._blind = bool(self.get_parameter('blind').value)
        self._t0 = None
        self._last = None
        self.report = NodeReport(self.get_parameter('report_file').value, 'spoof', {
            'scenario': scenario.name if scenario else '',
            'output_topic': self.get_parameter('output_topic').value})
        self._out = self.create_publisher(JointState, self.get_parameter('output_topic').value,
                                          10)
        self._listen = self.create_subscription(
            JointReference, self.get_parameter('listen_topic').value, self._on_trusted, 100)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().warn(f"spoofer up: publishing on "
                               f"{self.get_parameter('output_topic').value}")

    def _on_trusted(self, msg):
        if self._t0 is None:
            self._t0 = stamp_of(msg)
        self._last = msg
        self.report.count('received_trusted')

    def _tick(self):
        q = None
        msg = JointState()
        if self._last is not None:
            q = spoofed_state(self._schedule, stamp_of(self._last) - self._t0,
                              self._last.position)
            msg.header.stamp = self._last.header.stamp
        elif self._blind:
            q = self._blind_q
        if q is not None:
            msg.header.frame_id = MARK
            msg.name = list(JOINT_NAMES)
            msg.position = q.tolist()
            msg.velocity = [0.0] * len(JOINT_NAMES)
            self._out.publish(msg)
            self.report.count('spoofed')
        self.report.observe('output_subscribers', self._out.get_subscription_count())
        self.report.observe('listen_publishers', self._listen.get_publisher_count())
        self.report.maybe_write()


def main(args=None):
    init(args=args)
    stop = StopFlag()
    node = JointStateSpoofer()
    try:
        spin_until_shutdown(node, stop=stop)
    finally:
        node.report.write(final=True)