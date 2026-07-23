import os
import time
import rclpy
import random
import subprocess
import ikpy.chain
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Int8
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TaskPlanner(Node):

    def __init__(self):
        super().__init__('task_planner')
        self.context_ = self.create_publisher(Int8, '/task_context', 10)
        self.get_logger().info("Expriment started")
        # Base physical parameters
        self.base_mass = 5.0
        self.base_inertia = 0.008333
        self.base_pose = "0.5 0.0 0.05 0 0 0"
        self.arm_chain = ikpy.chain.Chain.from_urdf_file(
            "/workspaces/thesis/iiwa.urdf",
            base_elements=["world"])
        # Expriment loop
        self.timer = self.create_timer(20.0, self.exp_loop)
        self.arm_ = self.create_publisher(JointTrajectory, '/iiwa_arm_controller/joint_trajectory', 10)

    def calc_ik(self, x, y, z):
        target_position = [x, y, z]
        target_orientation = [0,0,-1] # Forcing straight down to floor
        ik_sol = self.arm_chain.inverse_kinematics(
            target_position= target_position,
            target_orientation=target_orientation,
            orientation_mode="Z"
        )
        return ik_sol[3:10].tolist()
    
    def move_arm(self, target_pos, durr_sec=5):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 
            'joint_a5', 'joint_a6', 'joint_a7'
        ]
        point = JointTrajectoryPoint()
        point.positions = target_pos
        point.time_from_start = Duration(sec=durr_sec, nanosec=0) # movement in 2 seconds
        traj_msg.points.append(point)
        self.arm_.publish(traj_msg)

    def grab_cube(self):
        self.get_logger().info("Magnet ON - Cube Grabbed")
        os.system("gz topic -t /gripper/attach -m gz.msgs.Empty -p ' '")
    
    def drop_cube(self):
        self.get_logger().info("Magnet OFF - Cube Dropped")
        os.system("gz topic -t /gripper/detach -m gz.msgs.Empty -p ' '")
    
    def generate_payload_spawn(self):
        # Randomizing the mass and inertia
        rand_mass = random.uniform(4.95, 5.06)
        scale = rand_mass / self.base_mass
        rand_inertia = self.base_inertia * scale
        self.get_logger().info(f"Generating new workpiece -> Mass: {rand_mass:.3f} kg | Inertia: {rand_inertia:.6f}")
        # Creating a sdf for different mass
        sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
    <model name="payload cube">
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
        # Removing the old playload
        subprocess.run(
            "gz service -s /world/empty/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --req 'name: \"payload\", type: MODEL'",
            shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        # Appearing the new payload
        subprocess.run(
            f"gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --req 'sdf_filename: \"{file_path}\", name: \"payload\"'",
            shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def exp_loop(self):
        # Without Payload
        self.publish_context(0)
        self.get_logger().info("Arm is moving back to Sources...")
        hover_angles = self.calc_ik(0.5,0.0,0.25)
        self.move_arm(hover_angles, 4)
        time.sleep(4.5)
        self.get_logger().info("Grabbing cube")
        grab_anlges = self.calc_ik(0.5,0.0,0.10)
        self.move_arm(grab_anlges, 2)
        time.sleep(2.5)
        self.grab_cube()
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
        time.sleep(0.5)
        # Drop
        self.publish_context(0)
        self.get_logger().info("Arm dropped the Payload...")
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
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