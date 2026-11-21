from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ids_bringup.campaign import load_campaign, resolve
from ids_bringup.runs import BAG_TOPICS, prepare_run


def _truthy(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _run(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('ids_bringup')
    campaign = load_campaign(arg('campaign') or os.path.join(share, 'config', 'campaign.yaml'))
    scenario = resolve(campaign, arg('scenario'), int(arg('seed')))
    files = prepare_run(arg('output_dir'), scenario,
                        os.path.join(get_package_share_directory('iiwa_sim'), 'config',
                                     'controllers.yaml'),
                        _truthy(arg('halt_on_alarm')), datetime.now())
    planner = scenario['planner']
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'friction_profile': scenario['friction_profile'],
                          'controllers_file': files.controllers_file,
                          'payload_mass': str(float(planner['payload_mass'])),
                          'gui': arg('gui')}.items())
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--storage', 'mcap', '--use-sim-time',
             '-o', files.bag_directory, '--topics', *BAG_TOPICS],
        output='log')
    plc = Node(package='ids_experiments', executable='plc_planner', name='plc_planner',
               parameters=[{'use_sim_time': True, 'cycles': int(planner['cycles']),
                            'payload_mass': float(planner['payload_mass'])}],
               output='screen', emulate_tty=True)
    mitm = Node(package='ids_experiments', executable='mitm', name='mitm',
                parameters=[{'use_sim_time': True, 'scenario_file': files.scenario_file}],
                output='screen', emulate_tty=True)
    supervisor = Node(package='ids_bringup', executable='run_supervisor', name='run_supervisor',
                      parameters=[{'use_sim_time': True,
                                   'duration': float(scenario['duration']),
                                   'summary_file': files.summary_file}],
                      output='screen', emulate_tty=True)
    return [
        LogInfo(msg=f"run {scenario['name']} seed {scenario['seed']} -> {files.directory}"),
        sim, record, plc, mitm, supervisor,
        RegisterEventHandler(OnProcessExit(
            target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='run finished'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='smoke'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('campaign', default_value=''),
        DeclareLaunchArgument('output_dir', default_value='/ws/data/runs'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('halt_on_alarm', default_value='false'),
        OpaqueFunction(function=_run),
    ])