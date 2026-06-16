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

    config_controller = os.path.join(
        get_package_share_directory('segway_control'),
        'config', 'balance_controller.yaml'
    )

    balance_controller_node = Node(
        package='segway_control',
        executable='balance_controller',
        name='balance_controller',
        output='screen',
        parameters=[config_controller],
    )

    config_imu = os.path.join(
        get_package_share_directory('segway_control'),
        'config', 'imu_filter.yaml'
    )

    imu_filter_node = Node(
        package='segway_control',
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[config_imu, {'use_sim_time': True}],
    )

    return LaunchDescription([
        imu_filter_node,
        balance_controller_node,
    ])