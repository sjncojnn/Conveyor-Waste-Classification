#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directories
    sim_pkg      = get_package_share_directory('my_conveyor_gazebo')
    bot_desc_pkg = get_package_share_directory('yolobot_description')
    bot_ctrl_pkg = get_package_share_directory('yolobot_control')
    yolo_pkg     = get_package_share_directory('yolobot_recognition')


    # 2) After a short delay, spawn YOLObot URDF into that same Gazebo
    spawn_bot = TimerAction(
        period=4.0,  # seconds to wait for gzserver and controllers to be ready
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bot_desc_pkg,
                             'launch',
                             'spawn_yolobot_launch.launch.py')
            )
        )]
    )

    # 3) Bring up the base‐drive controller for YOLObot
    bot_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_ctrl_pkg,
                         'launch',
                         'yolobot_control.launch.py')
        )
    )

    # 4) Start the YOLOv8 inference node
    yolo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_pkg,
                         'launch',
                         'launch_yolov8.launch.py')
        )
    )


    return LaunchDescription([
        spawn_bot,   # injects your YOLObot into that simulation
        bot_ctrl,    # starts the drive‐base node
        yolo_node,   # starts the AI inference node
    ])
