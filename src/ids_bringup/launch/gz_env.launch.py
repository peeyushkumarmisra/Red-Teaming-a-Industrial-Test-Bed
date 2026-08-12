import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.actions import OpaqueFunction, DeclareLaunchArgument, SetLaunchConfiguration

# Pick URDF based on enable_drift
def pick_urdf(context):
    drift = LaunchConfiguration('enable_drift').perform(context)
    if drift.lower() == 'true':
        urdf_path = '/workspaces/thesis/iiwa_friction.urdf'
    else:
        urdf_path = '/workspaces/thesis/iiwa.urdf'
    return [SetLaunchConfiguration('urdf_path', urdf_path)]

def generate_launch_description():
    # Drift is enabled
    declare_drift_cmd = DeclareLaunchArgument(
        'enable_drift',
        default_value= 'false',
        description="Enabling friction drift in Gazebo (Experiment 2)")
    # URDF selector
    urdf_selector = OpaqueFunction(function=pick_urdf)
    urdf_path_config = LaunchConfiguration('urdf_path')
    
    # Starting Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 
                'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', '-r empty.sdf')]
    )

    # Starting Robot State Publisher
    robot_state_pub = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        arguments = [urdf_path_config],
        parameters = [{'use_sim_time' : True}])

    # Spawning bot in Gazeebo
    spawn_robot = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        output = 'screen',
        arguments = ['-file', urdf_path_config, '-name', 'iiwa', '-z', '0.0'])

    # Delaying Spawning Controllers
    spawn_controllers = TimerAction(
        period = 5.0,
        actions = [
            ExecuteProcess(
                cmd     = ['ros2', 'run', 'controller_manager', 'spawner',
                           'joint_state_broadcaster'],
                output  = 'screen'
            ),
            ExecuteProcess(
                cmd     = ['ros2', 'run', 'controller_manager', 'spawner',
                           'effort_controller'],
                output  = 'screen'
            )
        ]
    )
    return LaunchDescription([
        declare_drift_cmd, urdf_selector, gz_sim, 
        robot_state_pub, spawn_robot, spawn_controllers])