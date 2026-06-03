"""
balance_controller.launch.py
Lance la boucle interne PID d'équilibre.
À utiliser avec imu_filter.launch.py et gazebo.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('segway_control'),
        'config', 'balance_controller.yaml'
    )

    balance_controller = Node(
        package='segway_control',
        executable='balance_controller',
        name='balance_controller',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([balance_controller])
