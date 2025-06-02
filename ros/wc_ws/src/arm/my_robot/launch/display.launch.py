from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import subprocess

def generate_launch_description():
    # Đường dẫn tới file Xacro
    xacro_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'my_conveyor_belt.xacro')  # Cập nhật tên file

    # Chuyển đổi Xacro thành URDF
    robot_desc = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')

    # Đường dẫn tới file cấu hình RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('my_robot'),
        'rviz', 'robot_display.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])