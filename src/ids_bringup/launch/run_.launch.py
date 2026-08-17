import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnShutdown
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.actions import ExecuteProcess, RegisterEventHandler


# Config
URDF                = '/workspaces/thesis/iiwa.urdf'
PLOT_SCRIPT         = '/workspaces/thesis/src/ids_bringup/launch/plot_csv.py'
# Timing (seconds)
CONTROLLER_DELAY    = 10.0      # Wait before spawning ros2 controllers
EXPERIMENT_DELAY    = 15.0      # Wait before starting task planner


def generate_launch_description():
    # Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 
                'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', '-r empty.sdf')]
    )
    # Bridging Gazebo's clock to ROS 2
    clock_bridge = Node(
        package     = 'ros_gz_bridge',
        executable  = 'parameter_bridge',
        arguments   = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output      = 'screen'
    )
    # Starting Robot State Publisher
    robot_state_pub = Node(
        package     = 'robot_state_publisher',
        executable  = 'robot_state_publisher',
        output      = 'screen',
        arguments   = [URDF],
        parameters  = [{'use_sim_time' : True}]
    )
    # Spawning bot in Gazeebo
    spawn_robot = Node(
        package     = 'ros_gz_sim',
        executable  = 'create',
        output      = 'screen',
        arguments   = ['-file', URDF, '-name', 'iiwa', '-z', '0.0']
    )
    # Spawning Controllers (With Delay)
    spawn_controllers = TimerAction(
        period  = CONTROLLER_DELAY,
        actions = [
            ExecuteProcess(
                cmd = ['ros2', 'run', 'controller_manager',
                       'spawner','joint_state_broadcaster'],
                output  = 'screen'),
            ExecuteProcess(
                cmd = ['ros2', 'run', 'controller_manager',
                       'spawner', 'effort_controller'],
                output  = 'screen') ]
    )
    # Task Planner (with further delay)
    planner_node = TimerAction(
        period   = EXPERIMENT_DELAY,
        actions  = [
            # The PLC / Task Planner
            Node(package    = 'ids_experiments',
                 executable = 'task_planner',
                 name       = 'task_planner_node',
                 output     = "screen"),
            # The Vulnerable Controller
            Node(package    = 'vulnerable_controller',
                 executable = 'vulnerable_controller',
                 name       = 'vulnerable_controller',
                 output     = 'screen'),
            # The Shadow Controller / IDS
            Node(package    = 'context_aware_ids',
                 executable = 'ids_node',
                 name       = 'ids_node',
                 output     = "screen",
                 parameters = [{'urdf_path': URDF}] ),
            # The Attacker Proxy
            Node(package    = 'vulnerable_controller',
                 executable = 'attacker_node',
                 name       = 'attacker_node', 
                 output     = 'screen' )
        ]
    )
    # Generating Plot (after sim has stopped)
    gen_plot = RegisterEventHandler(
        OnShutdown(
            on_shutdown = [
                LogInfo(msg=['SHUTDOWN COMPLETE. Running plot script']),
                ExecuteProcess(cmd=['python3', PLOT_SCRIPT], output="screen")
            ] 
        )
    )
    # Launch Description
    return LaunchDescription([gz_sim, clock_bridge, robot_state_pub, spawn_robot,
                              spawn_controllers, planner_node, gen_plot])