# =============================================================================
#  mitm_node.py - the man-in-the-middle attacker
# -----------------------------------------------------------------------------
#  Sits on the path from the PLC to the robot: /plc/reference -> /robot/reference.
#  It forwards every sample unchanged until a scheduled attack is active, then
#  falsifies one joint's setpoint. Attack times count from the first reference it
#  sees. For every sample it also publishes /attack/truth (evaluation only).
#  Threat model: an implant on the control network that cannot reach the trusted
#  monitor network, where the IDS taps the original reference.
# =============================================================================

import rclpy
from rclpy.node import Node

from ids_experiments.attacks import AttackSchedule
from ids_experiments.scenario import load_scenario
from ids_experiments.shutdown import spin_until_shutdown
from ids_interfaces.msg import AttackTruth, JointReference


# Parameter scenario_file: empty means transparent forwarding.
class SetpointMitm(Node):

    def __init__(self):
        super().__init__('mitm')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('input_topic', '/plc/reference')
        self.declare_parameter('output_topic', '/robot/reference')
        self.declare_parameter('truth_topic', '/attack/truth')
        path = self.get_parameter('scenario_file').value
        attacks = load_scenario(path).attacks if path else ()
        self._schedule = AttackSchedule(attacks)
        self._t0 = None
        self._last = None
        self._output = self.create_publisher(
            JointReference, self.get_parameter('output_topic').value, 100)
        self._truth = self.create_publisher(
            AttackTruth, self.get_parameter('truth_topic').value, 100)
        self.create_subscription(JointReference, self.get_parameter('input_topic').value,
                                 self._forward, 100)
        for a in self._schedule.attacks:
            self.get_logger().info(
                f'event {a.event_id}: {a.kind} on a{a.joint + 1}, magnitude {a.magnitude}, '
                f'{a.start:.1f}-{a.end:.1f} s after the first reference')
        if not self._schedule.attacks:
            self.get_logger().info('no attacks scheduled: forwarding transparently')

    # Forward one sample, falsified if an attack is active, and report the truth.
    def _forward(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = stamp
        t = stamp - self._t0
        position, velocity, truth = self._schedule.apply(t, msg.position, msg.velocity)
        out = JointReference()
        # Keep the original stamp and sequence number, so the falsified sample looks genuine.
        out.header = msg.header
        out.seq = msg.seq
        out.position = position.tolist()
        out.velocity = velocity.tolist()
        out.acceleration = msg.acceleration
        out.payload_context = msg.payload_context
        out.payload_mass = msg.payload_mass
        self._output.publish(out)
        report = AttackTruth()
        report.header = msg.header
        report.implant_active = True
        report.attack_active = truth.attack_active
        report.kind = truth.kind
        report.joint_mask = list(truth.joint_mask)
        report.magnitude = float(truth.magnitude)
        report.event_id = int(truth.event_id)
        self._truth.publish(report)
        if truth.attack_active != self._last:
            self._last = truth.attack_active
            if truth.attack_active:
                self.get_logger().warn(f'event {truth.event_id} active at t={t:.2f} s')
            elif t > 0.0:
                self.get_logger().info(f'transparent at t={t:.2f} s')


def main(args=None):
    rclpy.init(args=args)
    spin_until_shutdown(SetpointMitm())
