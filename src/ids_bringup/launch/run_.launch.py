import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, DeclareLaunchArgument
from launch.actions import OpaqueFunction, SetLaunchConfiguration, ExecuteProcess, RegisterEventHandler


# Config
URDF_NORMAL         = '/workspaces/thesis/iiwa.urdf'
URDF_FRICTION       = '/workspaces/thesis/iiwa_friction.urdf'
PLOT_SCRIPT         = '/workspaces/thesis/src/ids_bringup/launch/plot_csv.py'

# Timing (seconds)
CONTROLLER_DELAY    = 10.0      # Wait before spawning ros2 controllers
EXPERIMENT_DELAY    = 15.0      # Wait before starting task planner


def my_exp(context):
    exp = LaunchConfiguration('exp').perform(context)
    if exp == 'exp2':
        urdf    = URDF_FRICTION
        drift   = 'true'
        attack  = 'false'
    elif exp == 'exp3':
        urdf    = URDF_FRICTION
        drift   = 'true'
        attack  = 'true'
    else:   # exp 1
        urdf    = URDF_NORMAL
        drift   = 'false'
        attack  = 'false'
    return [
        SetLaunchConfiguration('enable_drift', drift),
        SetLaunchConfiguration('enable_attack', attack),
        SetLaunchConfiguration('urdf_path', urdf),
        SetLaunchConfiguration('exp_name', exp),
    ]


def attack_controller(context):
    attack = LaunchConfiguration('enable_attack').perform
    if attack == 'true':
        return [Node(
            package = 'vulnerable_controller',
            executable = 'vulnerable_controller',
            name = 'vulnerable_controller',
            output = "screen")]
    else:
        return [Node(
            package = 'vulnerable_controller',
            executable = 'vulnerable_controller',
            name = 'vulnerable_controller',
            output = "screen",
            remappings = [('/spoofed_joint_states' , '/joint_states')] )]

def generate_launch_description():
    # Which Exp
    which_exp = DeclareLaunchArgument( 
        "exp", default_value= 'exp1',
        description='exp1 = baseline | exp2 = drift | exp3 = attack + drift'
    )

    # Settings based on experiments
    config_setup    = OpaqueFunction (function=my_exp)
    exp_name        = LaunchConfiguration('exp_name')
    urdf_path       = LaunchConfiguration('urdf_path')
    enable_attack   = LaunchConfiguration('enable_attack')

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
        arguments   = [urdf_path],
        parameters  = [{'use_sim_time' : True}]
    )

    # Spawning bot in Gazeebo
    spawn_robot = Node(
        package     = 'ros_gz_sim',
        executable  = 'create',
        output      = 'screen',
        arguments   = ['-file', urdf_path, '-name', 'iiwa', '-z', '0.0']
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
    exp_node = TimerAction(
        period  = EXPERIMENT_DELAY,
        actions = [
            # The PLC / Task Planner
            Node(package    = 'ids_experiments',
                 executable = 'task_planner',
                 name       = 'task_planner_node',
                 output     = "screen"),
            # The Vulnerable Controller
            OpaqueFunction(function=attack_controller),
            # The Shadow Controller / IDS
            Node(package    = 'context_aware_ids',
                 executable = 'ids_node',
                 name       = 'ids_node',
                 output     = "screen",
                 parameters = [{'urdf_path': URDF_NORMAL}] ),
            # The Attacker Proxy
            Node(package    = 'vulnerable_controller',
                 executable = 'attacker_node',
                 name       = 'attacker_node', 
                 output     = 'screen', 
                 condition  = IfCondition(enable_attack) )
        ]
    )

    # Generating Plot (after sim has stopped)
    gen_plot = RegisterEventHandler(
        OnShutdown(
            on_shutdown = [
                LogInfo(msg=[exp_name,'shutdown complete. Running plot script']),
                ExecuteProcess(cmd=['python3', PLOT_SCRIPT, exp_name], output="screen")
            ] 
        )
    )

    # Launch Description
    return LaunchDescription([which_exp, config_setup, gz_sim, clock_bridge,
                              robot_state_pub, spawn_robot, spawn_controllers,
                              exp_node, gen_plot])