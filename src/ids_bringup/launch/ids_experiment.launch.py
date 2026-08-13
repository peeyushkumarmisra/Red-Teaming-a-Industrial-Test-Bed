from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction

def launch_vulnerable_controller(context):
    attack = LaunchConfiguration('enable_attack').perform(context)
    if attack == 'true':
        return [Node(
            package = 'vulnerable_controller',
            executable = 'vulnerable_controller',
            name = 'vulnerable_controller',
            output = 'screen')]
    else:
        return [Node(
            package = 'vulnerable_controller',
            executable = 'vulnerable_controller',
            name = 'vulnerable_controller',
            output = 'screen',
            remappings=[('/spoofed_joint_states', '/joint_states')])]

def generate_launch_description():
    # Declaring Launch Arguments 
    enable_attack = LaunchConfiguration('enable_attack')
    declare_attack_cmd = DeclareLaunchArgument(
        'enable_attack', 
        default_value= 'false',
        description="For injecting MiM FDIA attack (Expriment 3)")

    # The PLC / Task Planner
    planner_node = Node(
        package = 'ids_experiments',
        executable = 'task_planner',
        name = 'task_planner_node',
        output = 'screen')

    # The Vulnerable Controller
    vulnerable_controller = OpaqueFunction(
        function=launch_vulnerable_controller)

    # The Shadow Controller / IDS
    ids_node = Node(
        package = 'context_aware_ids',
        executable = 'ids_node',
        name = 'ids_node',
        output = 'screen',
        parameters = [{'urdf_path': '/workspaces/thesis/iiwa.urdf'}])

    # The Attacker Proxy
    attacker_node = Node(
        package = 'vulnerable_controller',
        executable = 'attacker_node',
        name = 'attacker_node', 
        output = 'screen', 
        condition = IfCondition(enable_attack))

    # Build and Return the Launch Description
    return LaunchDescription ([
        declare_attack_cmd, planner_node, vulnerable_controller, 
        TimerAction(period=5.0, actions=[ids_node]), attacker_node])