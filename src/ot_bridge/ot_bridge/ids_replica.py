import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag
from ids_interfaces.msg import JointReference
from ot_bridge.modbus_io import read_command, TcpStore
from ot_bridge.modbus_map import COIL_IDS_ALARM
from ot_bridge.trajectory_generator import TrajectoryGenerator

HOME = [0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0]


class ReplicaCore:

    def __init__(self, start, payload_mass=5.0):
        self._generator = TrajectoryGenerator(start)
        self._mass = payload_mass

    def tick(self, t, store):
        command = read_command(store)
        position, velocity, acceleration = self._generator.step(t, command)
        return {'position': position, 'velocity': velocity, 'acceleration': acceleration,
                'context': int(command.context),
                'mass': self._mass if command.context else 0.0, 'seq': int(command.seq)}


class IdsReplica(Node):

    def __init__(self):
        super().__init__('ids_replica')
        self.declare_parameter('plc_host', '10.203.20.20')
        self.declare_parameter('plc_port', 502)
        self.declare_parameter('scan_period', 0.01)
        self.declare_parameter('payload_mass', 5.0)
        self.declare_parameter('start', HOME)
        self.declare_parameter('write_alarm_coil', True)
        start = np.array(self.get_parameter('start').value, dtype=float)
        self._core = ReplicaCore(start, float(self.get_parameter('payload_mass').value))
        self._store = TcpStore(self.get_parameter('plc_host').value,
                               int(self.get_parameter('plc_port').value))
        self._connected = self._store.connect()
        self._write_alarm = bool(self.get_parameter('write_alarm_coil').value)
        self._alarm = False
        self._warned = False
        self._reference_pub = self.create_publisher(JointReference, '/plc/reference', 100)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/ids/alarm', self._on_alarm, latched)
        self.create_timer(self.get_parameter('scan_period').value, self._scan)
        self.get_logger().info('IDS replica reading the trusted PLC mirror on the monitor net')

    def _on_alarm(self, msg):
        if bool(msg.data) != self._alarm:
            self._alarm = bool(msg.data)
            if self._write_alarm:
                try:
                    self._store.write_coil(COIL_IDS_ALARM, self._alarm)
                    self.get_logger().info(
                        f"wrote IDS alarm coil = {self._alarm} to the PLC")
                except (IOError, OSError) as error:
                    self.get_logger().warn(f'could not write the alarm coil: {error}')

    def _scan(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        try:
            out = self._core.tick(t, self._store)
        except (IOError, OSError) as error:
            if not self._warned:
                self._warned = True
                self.get_logger().warn(f'Modbus read failed, holding: {error}')
            return
        self._warned = False
        msg = JointReference()
        msg.header.stamp = now.to_msg()
        msg.seq = out['seq']
        msg.position = out['position'].tolist()
        msg.velocity = out['velocity'].tolist()
        msg.acceleration = out['acceleration'].tolist()
        msg.payload_context = out['context']
        msg.payload_mass = float(out['mass'])
        self._reference_pub.publish(msg)


def main(args=None):
    init(args=args)
    stop = StopFlag()
    spin_until_shutdown(IdsReplica(), stop=stop)