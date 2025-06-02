#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    get_pkg = get_package_share_directory

    # 1) Conveyor belt simulation (src/conveyor)
    conveyor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_pkg('conveyorbelt_gazebo'),
                'launch',
                'conveyorbelt.launch.py'
            )
        )
    )

    # 2) Conveyor pushers "robot" + controllers (src/arm)
    pushers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_pkg('my_conveyor_gazebo'),
                'launch',
                'simulate.launch.py'
            )
        )
    )

    # 3) Your custom controller node (src/arm/conveyor_controller)
    #    <-- subscribe to the YOLO cmd_vel topic
    controller_node = Node(
        package='conveyor_controller',
        executable='controller',
        name='arm_controller',
        output='screen',
        parameters=[{
            'subscribe_topic': '/yolobot/cmd_vel'
        }]
    )

    # 4) Camera spawn + base‐drive + YOLO inference (src/camera)
    yolobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_pkg('yolobot_gazebo'),
                'launch',
                'yolobot_launch.py'
            )
        )
    )

    # 5) Trash‐Bin spawner (src/trash_bin)
    trash_bin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_pkg('trash_bin'),
                'launch',
                'spawn_trash_bins.launch.py'
            )
        )
    )

    return LaunchDescription([
        conveyor_launch,
        pushers_launch,
        yolobot_launch,
        controller_node,
        trash_bin_launch,   # <— This line spawns the 5 trash bins
    ])
