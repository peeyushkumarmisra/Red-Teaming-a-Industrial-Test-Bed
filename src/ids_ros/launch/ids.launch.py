# =============================================================================
#  ids.launch.py - start the IDS instances
# -----------------------------------------------------------------------------
#  Starts /ids (v2) and, unless thesis:=false, /ids_thesis (dissertation design),
#  each with its YAML file. run_mode:=calibrate records a new calibration for both and
#  exits when done.
#  Example: ros2 launch ids_ros ids.launch.py run_mode:=calibrate
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Namespace and settings file of each IDS instance.
INSTANCES = (('ids', 'ids.yaml'), ('ids_thesis', 'ids_thesis.yaml'))


def _truthy(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


# Build the node list from the launch arguments.
def _nodes(context):
    config = os.path.join(get_package_share_directory('ids_ros'), 'config')
    mode = LaunchConfiguration('run_mode').perform(context)
    thesis = _truthy(LaunchConfiguration('thesis').perform(context))
    # In calibrate mode each instance exits after saving its calibration.
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


# Arguments: run_mode (armed | calibrate) and thesis (true | false).
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_mode', default_value='armed',
                              choices=['armed', 'calibrate']),
        DeclareLaunchArgument('thesis', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
