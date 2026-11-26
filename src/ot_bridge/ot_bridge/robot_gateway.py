import numpy as np
from rclpy.node import Node
from std_msgs.msg import Empty

from ids_experiments.shutdown import init, spin_until_shutdown, StopFlag
from ids_interfaces.msg import JointReference
from ot_bridge.modbus_io import read_command, TcpStore, write_status
from ot_bridge.modbus_map import COIL_SAFE_STOP
from ot_bridge.trajectory_generator import TrajectoryGenerator

HOME = [0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0]


class GatewayCore:

    def __init__(self, start, payload_mass=5.0):
        self._generator = TrajectoryGenerator(start)
        self._mass = payload_mass
        self._gripper = 0
        self._reference_seq = 0

    def tick(self, t, store):
        command = read_command(store)
        position, velocity, acceleration = self._generator.step(t, command)
        edge = None
        if command.gripper != self._gripper:
            edge = 'attach' if command.gripper else 'detach'
            self._gripper = command.gripper
        safe_stop = store.read_coil(COIL_SAFE_STOP)
        self._reference_seq = (self._reference_seq + 1) & 0xFFFF
        write_status(store, safe_stop, command.step, self._reference_seq)
        return {'position': position, 'velocity': velocity, 'acceleration': acceleration,
                'context': int(command.context),
                'mass': self._mass if command.context else 0.0,
                'seq': int(command.seq), 'gripper_edge': edge, 'safe_stop': safe_stop}


class RobotGateway(Node):

    def __init__(self):
        super().__init__('robot_gateway')
        self.declare_parameter('plc_host', '10.203.10.20')
        self.declare_parameter('plc_port', 502)
        self.declare_parameter('scan_period', 0.01)
        self.declare_parameter('payload_mass', 5.0)
        self.declare_parameter('start', HOME)
        start = np.array(self.get_parameter('start').value, dtype=float)
        self._core = GatewayCore(start, float(self.get_parameter('payload_mass').value))
        self._store = TcpStore(self.get_parameter('plc_host').value,
                               int(self.get_parameter('plc_port').value))
        self._connected = self._store.connect()
        self._reference_pub = self.create_publisher(JointReference, '/robot/reference', 100)
        self._attach_pub = self.create_publisher(Empty, '/gripper/attach', 10)
        self._detach_pub = self.create_publisher(Empty, '/gripper/detach', 10)
        self._warned = False
        self.create_timer(self.get_parameter('scan_period').value, self._scan)
        self.get_logger().info(
            f"robot gateway polling the PLC Modbus map at "
            f"{self.get_parameter('plc_host').value}:{self.get_parameter('plc_port').value}")

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
        if out['gripper_edge'] == 'attach':
            self._attach_pub.publish(Empty())
        elif out['gripper_edge'] == 'detach':
            self._detach_pub.publish(Empty())
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
    spin_until_shutdown(RobotGateway(), stop=stop)