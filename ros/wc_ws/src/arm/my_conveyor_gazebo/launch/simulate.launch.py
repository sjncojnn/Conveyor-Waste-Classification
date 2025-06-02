from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_my_conveyor_gazebo = FindPackageShare('my_conveyor_gazebo')
    pkg_my_robot = FindPackageShare('my_robot')

    # file xacro
    xacro_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'my_conveyor_belt.xacro'])

    # file world
    world_file = PathJoinSubstitution([pkg_my_conveyor_gazebo, 'worlds', 'empty.world'])

    # convert xacro to pdf
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])



    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn robot gazebo 
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_conveyor',
            '-x', '0.7', 
            '-y', '-4.5',  
            '-z', '0.9',  
        ],
        output='screen'
    )

    # Load controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            PathJoinSubstitution([pkg_my_conveyor_gazebo, 'config', 'controllers.yaml'])
        ],
        output='screen'
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn arm controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )
    delayed_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[arm_controller_spawner]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        delayed_joint_state_broadcaster_spawner,
        delayed_arm_controller_spawner
    ])