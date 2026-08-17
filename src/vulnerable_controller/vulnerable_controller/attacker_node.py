import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

class AttackerNode(Node):
    def __init__(self):
        super().__init__('attacker_node', parameter_overrides=[
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL, True)])
        self.get_logger().info("Malicious Attacker is ready to inject FDIA")
        # Listining to Ground Truth
        self.sub_truth          = self.create_subscription(
            JointState, '/joint_states', self.truth_callback, 10)
        # Publish Spoofed Data
        self.pub_spoofed        = self.create_publisher(
            JointState, '/spoofed_joint_states', 10)
        # Attack Timing Parameters
        self.start_time         = self.get_clock().now().nanoseconds / 1e9
        self.first_attack_at    = 80.0   # first injection at 80 s
        self.attack_interval    = 17.0   # then every 17 s
        self.attack_duration    = 0.5    # how long each injection lasts (seconds)
        self.next_attack_time   = self.start_time + self.first_attack_at
        self.attack_end_time    = 0.0
        self.attack_active      = False
        # Injecting offset to joint 4
        self.target_joint_index = 3
        self.attack_offset      = 5

    def truth_callback(self, msg):
        curr_time = self.get_clock().now().nanoseconds / 1e9
        # Start of attack
        if curr_time >= self.next_attack_time and not self.attack_active:
            self.attack_active = True
            self.attack_end_time = curr_time + self.attack_duration
            self.get_logger().warn(
                f"Executing FDIA Attack! Injecting {self.attack_offset}" 
                f"rad offset to Joint {self.target_joint_index + 1}.")
            self.next_attack_time += self.attack_interval
        # End of attack
        if self.attack_active and curr_time >= self.attack_end_time:
            self.attack_active = False
            self.get_logger().info(
                f"Attack window ended at t={curr_time:.2f}s.")
        # Creating Spoofed Message
        spoofed_msg         = JointState()
        spoofed_msg.header  = msg.header
        spoofed_msg.name    = msg.name
        # Converting tuples to lists 
        pos = list(msg.position)
        vel = list(msg.velocity)
        # Inject Attack
        if self.attack_active and len(pos) > self.target_joint_index:
            pos[self.target_joint_index] += self.attack_offset
        spoofed_msg.position = pos
        spoofed_msg.velocity = vel
        self.pub_spoofed.publish(spoofed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AttackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
