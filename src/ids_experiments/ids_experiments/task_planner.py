import os
import time
import rclpy
import random
import threading
import subprocess
import ikpy.chain
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Header
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TaskPlanner(Node):

    def __init__(self):
        super().__init__('task_planner', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.parameter.Parameter.Type.BOOL, True)])
        self.context_ = self.create_publisher(Int8, '/task_context', 10)
        self.arm_ = self.create_publisher(JointTrajectory,
                                          '/joint_trajectory', 10)
        self.joint_sub_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.latest_positions_ = [0.0] * 7
        self.get_logger().info("Expriment started")
        # Base physical parameters
        self.base_mass = 5.0
        self.base_inertia = 0.008333
        self.base_pose = "0.5 0.0 0.05 0 0 0"
        self.arm_chain = ikpy.chain.Chain.from_urdf_file(
            "/workspaces/thesis/iiwa.urdf",
            base_elements=["world"])
        self.is_first_spawn = True # Tto track the first spawn
        # Threading It
        self.loop_thread = threading.Thread(target=self.exp_loop)
        self.loop_thread.daemon = True
        self.loop_thread.start()

    def joint_state_callback(self, msg):
        # Map by name to maintain correct joint order
        name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        ordered = []
        for j in ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']:
            if j in name_to_pos:
                ordered.append(name_to_pos[j])
        if len(ordered) == 7:
            self.latest_positions_ = ordered

    def calc_ik(self, x, y, z):
        target_position = [x, y, z]
        target_orientation = [0,0,-1]
        ik_sol = self.arm_chain.inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation,
            orientation_mode="Z"
        )
        joint_angles = ik_sol[3:10].tolist()
        # Avoiding Singularity (joint_a4 is elbow joint)
        if abs(joint_angles[3]) < 0.20:
            self.get_logger().warn(f"Singularity detected (joint_a4={joint_angles[3]:.3f}), nudging target...")
            # Nudge target 3 cm higher and retry IK
            target_position[2] += 0.03
            ik_sol = self.arm_chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation,
                orientation_mode="Z"
            )
            joint_angles = ik_sol[3:10].tolist()
        return joint_angles
    
    def move_arm(self, target_pos, durr_sec=5):
        traj_msg = JointTrajectory()
        traj_msg.header = Header()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = [
            'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 
            'joint_a5', 'joint_a6', 'joint_a7'
        ]
        # Point 0: explicit current state at t=0 removes first-cycle ambiguity
        start_point = JointTrajectoryPoint()
        start_point.positions = self.latest_positions_
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        traj_msg.points.append(start_point)
        # Point 1: target state
        end_point = JointTrajectoryPoint()
        end_point.positions = target_pos
        end_point.time_from_start = Duration(sec=durr_sec, nanosec=0)
        traj_msg.points.append(end_point)
        self.arm_.publish(traj_msg)

    def grab_cube(self):
        self.get_logger().info("Magnet ON")
        result = subprocess.run(
            "gz topic -t /gripper/attach -m gz.msgs.Empty -p ' '",
            shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().warn(f"Attach failed: {result.stderr.strip()}")
        else:
            self.cube_attached = True

    def drop_cube(self):
        if not getattr(self, 'cube_attached', False):
            self.get_logger().warn("Drop called but cube was never attached — skipping detach")
            return
        self.get_logger().info("Magnet OFF")
        result = subprocess.run(
            "gz topic -t /gripper/detach -m gz.msgs.Empty -p ' '",
            shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().warn(f"Detach failed: {result.stderr.strip()}")
        self.cube_attached = False
    
    def generate_payload_spawn(self):
        # Randomizing the mass and inertia
        rand_mass = random.uniform(4.95, 5.06)
        scale = rand_mass / self.base_mass
        rand_inertia = self.base_inertia * scale
        self.get_logger().info(
            f"Generating new workpiece -> Mass: {rand_mass:.3f} kg | Inertia: {rand_inertia:.6f}")
        # Creating a sdf for different mass
        sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
    <model name="payload">
        <pose>0.5 0.0 0.05 0 0 0</pose>
        <link name="cube_link">
            <inertial>
                <mass>{rand_mass:.3f}</mass>
                <inertia>
                    <ixx>{rand_inertia:.6f}</ixx>
                    <iyy>{rand_inertia:.6f}</iyy>
                    <izz>{rand_inertia:.6f}</izz>
                    <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
                </inertia>
            </inertial>
            <visual name="visual">
                <geometry>
                    <box><size>0.1 0.1 0.1</size></box>
                </geometry>
                <material>
                    <ambient>0.8 0.1 0.1 1</ambient>
                    <diffuse>0.8 0.1 0.1 1</diffuse>
                </material>
            </visual>
            <collision name="collision">
                <geometry>
                    <box><size>0.1 0.1 0.1</size></box>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.0</mu>
                            <mu2>1.0</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
        </link>
    </model>
</sdf>"""
        # Temp File
        file_path = "/workspaces/thesis/payload.sdf"
        with open(file_path, "w") as f:
            f.write(sdf_content)
        # --- REMOVE OLD PAYLOAD ---
        if self.is_first_spawn:
            self.get_logger().info("First run: skipping removal.")
            self.is_first_spawn = False
        else:
            self.get_logger().info("Removing old payload...")
            cmd = ("gz service -s /world/empty/remove "
                   "--reqtype gz.msgs.Entity "
                   "--reptype gz.msgs.Boolean "
                   "--req 'name: \"payload\", type: MODEL'")
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            if result.returncode != 0 or "not found" in (result.stdout + result.stderr):
                self.get_logger().warn(
                    f"Removal failed (entity may already be gone): {result.stderr.strip()}")
            else:
                self.get_logger().info("Old payload removed.")
            time.sleep(0.5)

        # --- SPAWN NEW PAYLOAD ---
        self.get_logger().info("Spawning new payload...")
        cmd = (f"gz service -s /world/empty/create "
               f"--reqtype gz.msgs.EntityFactory "
               f"--reptype gz.msgs.Boolean "
               f"--req 'sdf_filename: \"{file_path}\", name: \"payload\"'")
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f"Spawn FAILED: {result.stderr.strip()}")
        else:
            self.get_logger().info("Spawn succeeded.")
        time.sleep(0.5)

    def exp_loop(self):
        time.sleep(3.0) # Moment to initialize before looping
        while rclpy.ok():
            # Without Payload
            self.publish_context(0)
            self.get_logger().info("Arm is moving back to Sources...")
            hover_angles = self.calc_ik(0.6,0.0,0.25)
            self.move_arm(hover_angles, 4)
            time.sleep(4.5)
            self.get_logger().info("Grabbing cube")
            grab_anlges = self.calc_ik(0.5,0.0,0.10)
            self.move_arm(grab_anlges, 2)
            time.sleep(2.2)
            time.sleep(0.3)
            self.grab_cube()
            self.cube_attached = True
            time.sleep(0.5)
            # With Payload
            self.publish_context(1)
            self.get_logger().info("Lifting cube...")
            self.move_arm(hover_angles, 2)
            time.sleep(2.5)
            self.get_logger().info("Arm is moving to Destination...")
            dest_angles = self.calc_ik(0.0, 0.5, 0.25)
            self.move_arm(dest_angles, 4)
            time.sleep(4.5)
            self.get_logger().info("Lowering to floor...")
            drop_angles = self.calc_ik(0.0, 0.5, 0.10)
            self.move_arm(drop_angles, 2)
            time.sleep(2.5)
            self.drop_cube()
            time.sleep(1.0)
            # Drop
            self.publish_context(0)
            self.get_logger().info("Arm dropped the Payload...")
            self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 5)
            time.sleep(5.5)
            # Respawing
            self.generate_payload_spawn()

    def publish_context(self, val):
        msg = Int8()
        msg.data = val
        self.context_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    # First cube before loop begin
    node.generate_payload_spawn()
    time.sleep(1.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Deleting sdf files
    if os.path.exists('/workspaces/thesis/payload.sdf'):
        os.remove('/workspaces/thesis/payload.sdf')
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()