"""
    @Author Jason Hughes
    @Date November 2025

    @Brief launch the serial connection to the jackal
    and the jackal velocity controller
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jackal_control'),
                'launch',
                'control.launch.py'
            ])
        ])
        )
    node = Node(
        package='jackal_serial',
        executable='jackal_serial_node',
        name='jackal_serial_node',
        output='screen',
        parameters=[
            {"dev": "/dev/jackal",
             "baud_rate": 115200}
        ],
    )

    return LaunchDescription([control_launch, node])
