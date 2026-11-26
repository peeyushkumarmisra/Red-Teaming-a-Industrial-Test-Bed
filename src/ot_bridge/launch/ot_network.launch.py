from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    plc_host = LaunchConfiguration('plc_host')
    scan_period = LaunchConfiguration('scan_period')
    return LaunchDescription([
        DeclareLaunchArgument('plc_host', default_value='10.203.10.66'),
        DeclareLaunchArgument('scan_period', default_value='0.01'),
        Node(package='ot_bridge', executable='robot_gateway', name='robot_gateway',
             output='screen',
             parameters=[{'plc_host': plc_host, 'scan_period': scan_period,
                          'use_sim_time': True}]),
    ])