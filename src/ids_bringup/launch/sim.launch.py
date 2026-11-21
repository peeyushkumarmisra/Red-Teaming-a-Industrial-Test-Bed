# =============================================================================
#  sim.launch.py - Gazebo, robot, controllers, bridges and the part
# -----------------------------------------------------------------------------
#  Order of start-up: stop any leftover Gazebo -> Gazebo (headless unless gui:=true) ->
#  robot description and spawn -> joint state broadcaster -> tracking controller ->
#  payload cube. The part is spawned only after the controller is active, so the
#  gripper's automatic attach happens with the arm held at home (B16, B21).
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from ids_bringup.runs import PAYLOAD_POSE, stop_gazebo
from ids_experiments.payload import payload_sdf

WORLD = 'cell'


def _truthy(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


# Load and activate one controller through controller_manager.
def _spawner(controller):
    return Node(package='controller_manager', executable='spawner',
                arguments=[controller, '--controller-manager', '/controller_manager',
                           '--controller-manager-timeout', '120'],
                output='screen')


def _sim(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # A leftover server would capture this run's controllers (B20).
    leftover = stop_gazebo()
    sim_share = get_package_share_directory('iiwa_sim')
    controllers = arg('controllers_file') or os.path.join(sim_share, 'config', 'controllers.yaml')
    # Expand the robot at launch time with this run's friction profile and controllers.
    description = xacro.process_file(
        os.path.join(sim_share, 'urdf', 'iiwa_sim.urdf.xacro'),
        mappings={'friction_profile': arg('friction_profile'),
                  'controllers_file': controllers}).toxml()
    world = os.path.join(sim_share, 'worlds', 'cell.sdf')
    server_only = '' if _truthy(arg('gui')) else '-s '
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 2 {server_only}{world}',
                          'on_exit_shutdown': 'true'}.items())
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                           parameters=[{'robot_description': description,
                                        'use_sim_time': True}],
                           output='log')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                  parameters=[{'config_file': os.path.join(
                      get_package_share_directory('ids_bringup'), 'config', 'bridge.yaml')}],
                  output='log')
    # Service bridge for set_pose, used to re-seat the part at a station.
    services = Node(package='ros_gz_bridge', executable='parameter_bridge',
                    name='gz_service_bridge',
                    arguments=[f'/world/{WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
                    output='log')
    spawn_robot = Node(package='ros_gz_sim', executable='create', name='spawn_robot',
                       parameters=[{'world': WORLD, 'topic': 'robot_description',
                                    'name': 'iiwa'}],
                       output='screen')
    broadcaster = _spawner('joint_state_broadcaster')
    controller = _spawner('reference_tracking_controller')
    actions = [
        LogInfo(msg=f'stopped Gazebo left over from an earlier run: pids {leftover}'
                if leftover else 'no Gazebo left over from an earlier run'),
        gazebo, state_publisher, bridge, services, spawn_robot,
        # Strict chain: each step starts when the previous one has finished.
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[controller])),
    ]
    mass = float(arg('payload_mass'))
    # Spawn the part after the controller is active (see the header).
    if mass > 0.0:
        x, y, z = PAYLOAD_POSE
        payload = Node(package='ros_gz_sim', executable='create', name='spawn_payload',
                       parameters=[{'world': WORLD, 'string': payload_sdf(mass),
                                    'name': 'payload', 'x': x, 'y': y, 'z': z}],
                       output='screen')
        actions.append(RegisterEventHandler(OnProcessExit(target_action=controller,
                                                          on_exit=[payload])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('friction_profile', default_value='wear',
                              choices=['none', 'nominal', 'wear', 'thesis']),
        DeclareLaunchArgument('controllers_file', default_value=''),
        DeclareLaunchArgument('payload_mass', default_value='5.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        OpaqueFunction(function=_sim),
    ])
