#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_trash = FindPackageShare('trash_bin')
    trash_bin_xacro = PathJoinSubstitution([pkg_trash, 'urdf', 'trash_bin.xacro'])

    trash_bin_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='trash_bin_state_publisher',
        namespace='trash',  # → publishes to /trash/robot_description
        output='screen',
        parameters=[{
            'robot_description': Command([FindExecutable(name='xacro'), ' ', trash_bin_xacro]),
            'use_sim_time': True
        }]
    )

    def make_trash_spawn(entity_name, x, y, z, roll, pitch, yaw):
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic',           'trash/robot_description',
                '-entity',          entity_name,
                '-x',               str(x),
                '-y',               str(y),
                '-z',               str(z),
                '-R',               str(roll),
                '-P',               str(pitch),
                '-Y',               str(yaw),
                '-reference_frame', 'world'
            ],
            output='screen'
        )
    
    trash_positions = [
        ('trash_bin_1', -1.15, -1.20, 0.0, 0.0, 0.0, -1.566415),
        ('trash_bin_2', -1.15,  0.40, 0.0, 0.0, 0.0, -1.566415),
        ('trash_bin_3', -1.15,  2.00, 0.0, 0.0, 0.0, -1.566415),
        ('trash_bin_4', -1.15,  3.60, 0.0, 0.0, 0.0, -1.566415),
        ('trash_bin_5',  0.00,  5.20, 0.0, 0.0, 0.0,  0.0     ),
    ]
    spawn_handlers = []
    for idx, (ename, ex, ey, ez, eroll, epitch, eyaw) in enumerate(trash_positions):
        spawn_node = make_trash_spawn(ename, ex, ey, ez, eroll, epitch, eyaw)

        # Delay each spawn by 0.1 * idx seconds so they fire in sequence
        handler = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=trash_bin_state_publisher,
                on_start=[
                    TimerAction(period=0.1 * idx, actions=[spawn_node])
                ]
            )
        )
        spawn_handlers.append(handler)

    return LaunchDescription([
        trash_bin_state_publisher,
        *spawn_handlers
    ])
