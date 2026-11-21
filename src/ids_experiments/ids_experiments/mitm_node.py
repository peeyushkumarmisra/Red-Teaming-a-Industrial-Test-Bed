# =============================================================================
#  mitm_node.py - the man-in-the-middle attacker
# -----------------------------------------------------------------------------
#  Sits on the path from the PLC to the robot: /plc/reference -> /robot/reference.
#  It forwards every sample unchanged until a scheduled attack is active, then
#  falsifies the setpoints of the attacked joints with one of the ten attack kinds of
#  ids_attacks (step, ramp, sine, scaling, freeze, replay, delay, noise, coordinated,
#  drift_mimic). Attack times count from the first reference it sees. For every sample
#  it also publishes /attack/truth (evaluation only).
#  Torque-report cover (cover_torque: true, v1.4): the controller then publishes its
#  measurements on /robot/measurement_raw, and this node relays them to
#  /robot/measurement with effort_applied changed to the torque the trusted reference
#  would have produced. The attacker thus hides the command-torque evidence; the IDS
#  must still see the attack in the motion itself.
#  Threat model: an implant on the control network that cannot reach the trusted
#  monitor network, where the IDS taps the original reference.
# =============================================================================

import numpy as np
import rclpy
from rclpy.node import Node

from ids_attacks.cover import IIWA_KD, IIWA_KP, TorqueCover
from ids_attacks.library import AttackSchedule
from ids_experiments.scenario import load_scenario
from ids_experiments.shutdown import spin_until_shutdown
from ids_interfaces.msg import AttackTruth, JointMeasurement, JointReference


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# Parameter scenario_file: empty means transparent forwarding. kp, kd and reference_hold
# describe the robot's controller, which the cover needs to compute a believable torque.
class SetpointMitm(Node):

    def __init__(self):
        super().__init__('mitm')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('input_topic', '/plc/reference')
        self.declare_parameter('output_topic', '/robot/reference')
        self.declare_parameter('truth_topic', '/attack/truth')
        self.declare_parameter('measurement_input', '/robot/measurement_raw')
        self.declare_parameter('measurement_output', '/robot/measurement')
        self.declare_parameter('kp', list(IIWA_KP))
        self.declare_parameter('kd', list(IIWA_KD))
        self.declare_parameter('reference_hold', 0.05)
        path = self.get_parameter('scenario_file').value
        scenario = load_scenario(path) if path else None
        self._schedule = AttackSchedule(scenario.attacks if scenario else ())
        self._cover = None
        self._t0 = None
        self._last = None
        self._output = self.create_publisher(
            JointReference, self.get_parameter('output_topic').value, 100)
        self._truth = self.create_publisher(
            AttackTruth, self.get_parameter('truth_topic').value, 100)
        self.create_subscription(JointReference, self.get_parameter('input_topic').value,
                                 self._forward, 100)
        # Only a covered scenario relays measurements; otherwise the controller publishes
        # /robot/measurement itself.
        if scenario is not None and scenario.cover_torque:
            self._cover = TorqueCover(self.get_parameter('kp').value,
                                      self.get_parameter('kd').value,
                                      self.get_parameter('reference_hold').value)
            self._measurement = self.create_publisher(
                JointMeasurement, self.get_parameter('measurement_output').value, 100)
            self.create_subscription(JointMeasurement,
                                     self.get_parameter('measurement_input').value,
                                     self._relay, 100)
            self.get_logger().info('torque-report cover active: relaying measurements')
        for a in self._schedule.attacks:
            joints = ' '.join(f'a{j + 1}' for j in a.joints)
            self.get_logger().info(
                f'event {a.event_id}: {a.kind} on {joints}, magnitude {a.magnitude}, '
                f'{a.start:.1f}-{a.end:.1f} s after the first reference')
        if not self._schedule.attacks:
            self.get_logger().info('no attacks scheduled: forwarding transparently')

    # Forward one sample, falsified if an attack is active, and report the truth.
    def _forward(self, msg):
        stamp = stamp_of(msg)
        if self._t0 is None:
            self._t0 = stamp
        t = stamp - self._t0
        position, velocity, acceleration, truth = self._schedule.apply(
            t, msg.position, msg.velocity, msg.acceleration)
        if self._cover is not None:
            # Remember how far this sample was falsified, so the matching measurement's torque
            # can be corrected later.
            self._cover.remember(msg.seq, stamp, position - np.asarray(msg.position),
                                 velocity - np.asarray(msg.velocity))
        out = JointReference()
        # Keep the original stamp and sequence number, so the falsified sample looks genuine.
        out.header = msg.header
        out.seq = msg.seq
        out.position = position.tolist()
        out.velocity = velocity.tolist()
        out.acceleration = acceleration.tolist()
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

    # Cover: replace the reported torque by the torque of the trusted reference, matched
    #        through reference_seq.
    def _relay(self, msg):
        msg.effort_applied = self._cover.falsify(msg.effort_applied, msg.reference_seq,
                                                 stamp_of(msg)).tolist()
        self._measurement.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    spin_until_shutdown(SetpointMitm())
