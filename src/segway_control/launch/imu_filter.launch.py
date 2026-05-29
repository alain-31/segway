"""
imu_filter.launch.py
Lance uniquement le nœud de filtre IMU.
À inclure dans sim.launch.py et real.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('segway_control'),
        'config', 'imu_filter.yaml'
    )

    imu_filter_node = Node(
        package='segway_control',
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[config, {'use_sim_time': True}],
    )

    return LaunchDescription([imu_filter_node])