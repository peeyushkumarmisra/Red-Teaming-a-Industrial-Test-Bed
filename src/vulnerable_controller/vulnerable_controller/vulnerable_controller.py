import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState

class VulnerableController(Node):
    def __init__(self):
        super().__init__('vulnerable_controller', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', 
                                      rclpy.parameter.Parameter.Type.BOOL, True)])
        self.get_logger().info('Vulnerable Controller Booted')
        # Subscribing to trajectory from task planner
        self.sub_traj = self.create_subscription(
            JointTrajectory, '/joint_trajectory',
            self.trajectory_callback, 10)
        # Subscribing to spoofed feedback from attacker
        self.sub_feedback = self.create_subscription(
            JointState, '/spoofed_joint_states',
            self.feedback_callback, 10)
        # Command torque to robot (via effort_controller)
        self.pub_command = self.create_publisher(
            Float64MultiArray, '/effort_controllers/commands', 10)
        # Publishing ACTUAL torque to IDS (closes security blind spot)
        self.pub_torque = self.create_publisher(
            Float64MultiArray, '/vulnerable_controller/torque', 10)
        # Publishing controller state to IDS (reference q, qd, qdd)
        self.pub_state = self.create_publisher(
            JointTrajectoryControllerState,
            '/vulnerable_controller/controller_state', 10)
        # 1000 hz Control Loop
        self.timer = self.create_timer(0.001, self.control_loop)
        # State 
        self.trajectory = None
        self.trajectory_start_time = 0.0
        self.joint_names = [
            'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4',
            'joint_a5', 'joint_a6', 'joint_a7'
        ]
        self.m_q = np.zeros(7)
        self.m_qd = np.zeros(7)
        self.feedback_ready = False
        # PD Gains (ATTACK)
        self.Kp = np.array([800.0, 800.0, 800.0, 800.0, 500.0, 400.0, 100.0])
        self.Kd = np.array([80.0, 80.0, 80.0, 80.0, 50.0, 40.0, 10.0])

    def trajectory_callback(self, msg):
        self.trajectory = msg
        self.trajectory_start_time = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.joint_names = msg.joint_names
        self.get_logger().info('Received new trajectory')

    def feedback_callback(self, msg):
        # Mapping by name to maintain correct joint order
        name_to_q = {}
        name_to_qd = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                name_to_q[name] = msg.position[i]
            if i < len(msg.velocity):
                name_to_qd[name] = msg.velocity[i]
        for j, name in enumerate(self.joint_names):
            if name in name_to_q:
                self.m_q[j] = name_to_q[name]
            if name in name_to_qd:
                self.m_qd[j] = name_to_qd[name]
        self.feedback_ready = True

    def get_reference(self, t):
        # Linear interpolation of trajectory
        if self.trajectory is None or len(self.trajectory.points) == 0:
            return None, None, None
        # Compute absolute times for each point
        abs_times = []
        for p in self.trajectory.points:
            dt = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            abs_times.append(self.trajectory_start_time + dt)
        # Before first point
        if t <= abs_times[0]:
            q = np.array(self.trajectory.points[0].positions)
            return q, np.zeros(7), np.zeros(7)
        # During trajectory
        for i in range(len(abs_times) - 1):
            if abs_times[i] <= t <= abs_times[i + 1]:
                dt_seg = abs_times[i + 1] - abs_times[i]
                alpha = 0.0 if dt_seg < 1e-6 else (t - abs_times[i]) / dt_seg
                p0 = np.array(self.trajectory.points[i].positions)
                p1 = np.array(self.trajectory.points[i + 1].positions)
                q = (1.0 - alpha) * p0 + alpha * p1
                qd = (p1 - p0) / dt_seg if dt_seg > 1e-6 else np.zeros(7)
                qdd = np.zeros(7)
                return q, qd, qdd
        # After last point
        last = self.trajectory.points[-1]
        return np.array(last.positions), np.zeros(7), np.zeros(7)

    def control_loop(self):
        if not self.feedback_ready or self.trajectory is None:
            return
        t_now = self.get_clock().now().nanoseconds / 1e9
        q_ref, qd_ref, qdd_ref = self.get_reference(t_now)
        if q_ref is None:
            return
        # PD Control Law (uses SPOOFED feedback)
        p_error = q_ref - self.m_q
        d_error = qd_ref - self.m_qd
        tau_cmd = (self.Kp * p_error) + (self.Kd * d_error)
        # Publish to robot
        cmd_msg         = Float64MultiArray()
        cmd_msg.data    = tau_cmd.tolist()
        self.pub_command.publish(cmd_msg)
        # Publish ACTUAL torque to IDS (this is the critical fix)
        torque_msg      = Float64MultiArray()
        torque_msg.data = tau_cmd.tolist()
        self.pub_torque.publish(torque_msg)
        # Publish controller state to IDS
        state_msg                           = JointTrajectoryControllerState()
        state_msg.header.stamp              = self.get_clock().now().to_msg()
        state_msg.joint_names               = self.joint_names
        state_msg.reference.positions       = q_ref.tolist()
        state_msg.reference.velocities      = qd_ref.tolist()
        state_msg.reference.accelerations   = qdd_ref.tolist()
        self.pub_state.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VulnerableController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()