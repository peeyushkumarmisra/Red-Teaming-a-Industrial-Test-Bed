import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INSTANCES = (('ids', 'ids.yaml'), ('ids_thesis', 'ids_thesis.yaml'))


def _truthy(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _nodes(context):
    config = os.path.join(get_package_share_directory('ids_ros'), 'config')
    mode = LaunchConfiguration('run_mode').perform(context)
    thesis = _truthy(LaunchConfiguration('thesis').perform(context))
    overrides = {'run_mode': mode, 'exit_after_calibration': mode == 'calibrate'}
    nodes = []
    for namespace, config_file in INSTANCES:
        if namespace == 'ids_thesis' and not thesis:
            continue
        nodes.append(Node(package='ids_ros', executable='ids_node', name='ids_node',
                          namespace=namespace,
                          parameters=[os.path.join(config, config_file), overrides],
                          output='screen', emulate_tty=True))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_mode', default_value='armed',
                              choices=['armed', 'calibrate']),
        DeclareLaunchArgument('thesis', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])