import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'inspire_hand_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'inspire_hand.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = {'robot_description': f.read()}

    # Node for robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'inspire_hand'],
        output='screen'
    )

    # Spawner for the joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawner for the main controller
    spawn_inspire_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['inspire_hand_controller', '--controller-manager', '/controller_manager'],
    )

    # Ensure controllers are loaded only after the robot is spawned
    on_spawn_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_joint_state_broadcaster],
        )
    )

    on_broadcaster_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_inspire_hand_controller],
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
        on_spawn_exit,
        on_broadcaster_exit
    ])