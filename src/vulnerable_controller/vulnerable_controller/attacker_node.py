import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

class AttackerNode(Node):
    def __init__(self):
        super().__init__('attacker_node', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.parameter.Parameter.Type.BOOL, True)])
        self.get_logger().info("Malicious Attacker is ready to inject FDIA")
        # Listining to Ground Truth
        self.sub_truth      = self.create_subscription(
            JointState, '/joint_states', self.truth_callback, 10)
        # Publish Spoofed Data
        self.pub_spoofed    = self.create_publisher(
            JointState, '/spoofed_joint_states', 10)
        # Attack Parameters
        self.attack_active      = False
        self.attack_start_time  = self.get_clock().now().nanoseconds / 1e9
        self.attack_delay       = 60.0 # wait time
        # Injecting small offset to joint 4
        self.target_joint_index = 3
        self.attack_offset      = 0.05

    def truth_callback(self, msg):
        curr_time = self.get_clock().now().nanoseconds / 1e9
        # Trigger attack
        if curr_time - self.attack_start_time > self.attack_delay and not self.attack_active:
            self.attack_active = True
            self.get_logger().warn(
                f"Executing FDIA Attack! Injecting {self.attack_offset}" 
                f"rad offset to Joint {self.target_joint_index + 1}.")
        # Create Spoofed Message
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
