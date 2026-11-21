
import numpy as np
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from ids_experiments.kinematics import JOINTS, Kinematics, station_targets
from ids_experiments.payload import CUBE_SIZE
from ids_experiments.sequencer import Sequencer, shuttle_program
from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag
from ids_experiments.startup import StartupGate
from ids_experiments.trajectory import move_duration
from ids_interfaces.msg import JointReference

VELOCITY_LIMITS = np.array([1.4834, 1.4834, 1.7452, 1.3089, 2.2688, 2.356, 2.356])


class PlcPlanner(Node):

    def __init__(self):
        super().__init__('plc_planner')
        self.declare_parameter('scan_period', 0.01)
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('cycles', 0)
        self.declare_parameter('payload_mass', 5.0)
        self.declare_parameter('source', [-0.5, 0.0])
        self.declare_parameter('destination', [0.0, 0.5])
        self.declare_parameter('hover_height', 0.25)
        self.declare_parameter('grab_height', 0.10)
        self.declare_parameter('drop_height', 0.11)
        self.declare_parameter('travel_time', 4.0)
        self.declare_parameter('approach_time', 2.0)
        self.declare_parameter('dwell', 0.5)
        self.declare_parameter('speed_scale', 0.5)
        self.declare_parameter('max_acceleration', 2.0)
        self.declare_parameter('wait_for_gripper', True)
        self.declare_parameter('gripper_warning', 30.0)
        self.declare_parameter('seat_parts', True)
        self.declare_parameter('world', 'cell')
        self.declare_parameter('payload_name', 'payload')
        self.declare_parameter('mirror_domain', -1)
        self._joint_positions = None
        self._description = None
        self._sequencer = None
        self._gripper_state = None
        self._waiting_since = None
        self._warned = False
        self._gripper_warning = self.get_parameter('gripper_warning').value
        self._gate = StartupGate(self.get_parameter('start_delay').value,
                                 wait_for_gripper=self.get_parameter('wait_for_gripper').value)
        self._t0 = None
        self._seq = 0
        self._finished_logged = False
        self._seat_warned = False
        self._seat_client = self.create_client(
            SetEntityPose, f"/world/{self.get_parameter('world').value}/set_pose")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._reference_pub = self.create_publisher(JointReference, '/plc/reference', 100)
        self.extra_nodes = []
        self._mirror_pub = None
        mirror = int(self.get_parameter('mirror_domain').value)
        if mirror >= 0:
            context = Context()
            init(args=[], context=context, domain_id=mirror)
            node = Node('plc_mirror', context=context)
            self._mirror_pub = node.create_publisher(JointReference, '/plc/reference', 100)
            self.extra_nodes.append(node)
            self.get_logger().info(f'mirroring /plc/reference onto the monitor domain {mirror}')
        self._attach_pub = self.create_publisher(Empty, '/gripper/attach', 10)
        self._detach_pub = self.create_publisher(Empty, '/gripper/detach', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(String, '/robot_description', self._on_description, latched)
        self.create_subscription(String, '/gripper/state', self._on_gripper_state, 10)
        self.create_timer(self.get_parameter('scan_period').value, self._scan)
        self.get_logger().info('waiting for /joint_states, /robot_description and the gripper')

    def _on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in JOINTS):
            self._joint_positions = np.array([positions[j] for j in JOINTS])

    def _on_description(self, msg):
        self._description = msg.data

    def _on_gripper_state(self, msg):
        if msg.data != self._gripper_state:
            self.get_logger().info(f'gripper {msg.data}')
        self._gripper_state = msg.data

    def _prepare(self):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        kinematics = Kinematics(self._description)
        start = self._joint_positions.copy()
        targets = (station_targets('a', p('source'), p('hover_height'), p('grab_height'),
                                   p('drop_height'))
                   + station_targets('b', p('destination'), p('hover_height'),
                                     p('grab_height'), p('drop_height')))
        solutions = kinematics.chain(targets, start)
        stations = {s: {k: solutions[f'{s}_{k}'] for k in ('hover', 'grab', 'drop')}
                    for s in ('a', 'b')}
        limits = VELOCITY_LIMITS * p('speed_scale')
        accel = np.full(7, p('max_acceleration'))
        travel = max(p('travel_time'),
                     move_duration(stations['a']['hover'], stations['b']['hover'], limits, accel),
                     move_duration(start, stations['a']['hover'], limits, accel))
        approach = max(p('approach_time'),
                       move_duration(stations['a']['hover'], stations['a']['grab'], limits,
                                     accel))
        places = (tuple(p('destination')), tuple(p('source'))) if p('seat_parts') else None
        program = shuttle_program(stations['a'], stations['b'], p('payload_mass'), travel,
                                  approach, p('dwell'), places)
        self._sequencer = Sequencer(start, program, passes=p('cycles'))
        self.get_logger().info(
            f'program ready: {len(program)} steps per cycle, travel {travel:.2f} s, '
            f'approach {approach:.2f} s, cycles {p("cycles") or "unlimited"}')

    def _scan(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._sequencer is None:
            inputs = self._joint_positions is not None and self._description is not None
            detach, ready = self._gate.step(t, inputs, self._gripper_state)
            if detach:
                self._detach_pub.publish(Empty())
            if inputs and t > 0.0 and not self._gate.detached:
                self._waiting_since = self._waiting_since or t
                if not self._warned and t - self._waiting_since > self._gripper_warning:
                    self._warned = True
                    self.get_logger().warn('the gripper has not reported detached yet: check that '
                                           'the payload was spawned and the bridge is running')
            if ready:
                self._prepare()
            return
        if self._t0 is None:
            self._t0 = t
            self.get_logger().info(f'streaming references from t={t:.3f} s')
        out = self._sequencer.scan(t - self._t0)
        for command in out.commands:
            if command.name == 'seat':
                self._seat(command.args)
            elif command.name == 'attach':
                self._attach_pub.publish(Empty())
            else:
                self._detach_pub.publish(Empty())
        msg = JointReference()
        msg.header.stamp = now.to_msg()
        msg.seq = self._seq
        msg.position = out.position.tolist()
        msg.velocity = out.velocity.tolist()
        msg.acceleration = out.acceleration.tolist()
        msg.payload_context = int(out.context)
        msg.payload_mass = float(out.mass)
        if self._mirror_pub is not None:
            self._mirror_pub.publish(msg)
        self._reference_pub.publish(msg)
        self._seq += 1
        if out.finished and not self._finished_logged:
            self._finished_logged = True
            self.get_logger().info(f'program complete after {self._sequencer.completed_passes} '
                                   'cycles; holding the last pose')

    def _seat(self, xy):
        if not self._seat_client.service_is_ready():
            self._warn_seat('the set_pose service is not available: the part is not re-seated')
            return
        request = SetEntityPose.Request()
        request.entity.name = self.get_parameter('payload_name').value
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(xy[0])
        request.pose.position.y = float(xy[1])
        request.pose.position.z = CUBE_SIZE / 2.0
        request.pose.orientation.w = 1.0
        self._seat_client.call_async(request).add_done_callback(self._seated)

    def _seated(self, future):
        result = future.result()
        if result is None or not result.success:
            self._warn_seat('set_pose did not re-seat the part')

    def _warn_seat(self, text):
        if not self._seat_warned:
            self._seat_warned = True
            self.get_logger().warn(text)


def main(args=None):
    init(args=args)
    stop = StopFlag()
    planner = PlcPlanner()
    spin_until_shutdown(planner, *planner.extra_nodes, stop=stop)