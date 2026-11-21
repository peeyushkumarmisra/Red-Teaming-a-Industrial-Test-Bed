from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ids_bringup import cell
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
    cell_dir = arg('cell_dir')
    if _truthy(arg('require_agents')):
        missing = cell.missing_agents(cell_dir, scenario)
        if missing:
            raise RuntimeError(
                'no cell_agent heartbeat from ' + ', '.join(missing) + ': start '
                + ' and '.join(f'"ros2 run ids_bringup cell_agent {r}" in the {r} container'
                               for r in missing))
    files = prepare_run(arg('output_dir'), scenario,
                        os.path.join(get_package_share_directory('iiwa_sim'), 'config',
                                     'controllers.yaml'),
                        _truthy(arg('halt_on_alarm')), datetime.now())
    run_id = os.path.basename(files.directory)
    monitor_domain = int(arg('monitor_domain'))
    roles = cell.roles_for(scenario, monitor_domain)
    wires = cell.wiring(scenario, _truthy(arg('halt_on_alarm')))
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
    bridge = Node(package='ids_bringup', executable='monitor_bridge', name='monitor_bridge',
                  parameters=[{'config_file': os.path.join(share, 'config',
                                                           'monitor_bridge.yaml'),
                               'monitor_domain': monitor_domain}],
                  output='screen', emulate_tty=True)
    supervisor = Node(package='ids_bringup', executable='run_supervisor', name='run_supervisor',
                      parameters=[{'use_sim_time': True,
                                   'duration': float(scenario['duration']),
                                   'summary_file': files.summary_file}],
                      output='screen', emulate_tty=True)
    cell.publish_run(cell_dir, run_id, scenario, roles)

    def close(context):
        cell.close_run(cell_dir, run_id)
        return [LogInfo(msg=f'run {run_id} closed for the cell agents')]

    attacker = roles['attacker']
    return [
        LogInfo(msg=f"run {scenario['name']} seed {scenario['seed']} -> {files.directory}"),
        LogInfo(msg=f"robot listens on {wires['reference_topic']}; attacker "
                    f"{attacker['role']} ({attacker['credentials']}); halt on alarm "
                    f"{wires['halt_on_alarm']}"),
        sim, record, bridge, supervisor,
        RegisterEventHandler(OnProcessExit(
            target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='run finished'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=close)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='smoke'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('campaign', default_value=''),
        DeclareLaunchArgument('output_dir', default_value='/ws/data/runs'),
        DeclareLaunchArgument('cell_dir', default_value=cell.CELL_DIR),
        DeclareLaunchArgument('monitor_domain', default_value=str(cell.MONITOR_DOMAIN)),
        DeclareLaunchArgument('require_agents', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('halt_on_alarm', default_value='false'),
        OpaqueFunction(function=_run),
    ])