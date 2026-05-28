"""
gazebo.launch.py
Lance la simulation Gazebo Classic du segway miniature :
  1. Gazebo Classic avec le monde plat
  2. robot_state_publisher (URDF → TF)
  3. spawn_entity (spawner le robot dans Gazebo)

Topics publiés par Gazebo :
  /segway/imu/data          — IMU 100 Hz
  /segway/joint_states      — positions roues 50 Hz
  /segway/odom              — odométrie diff_drive

Topics attendus par Gazebo :
  /segway/cmd_vel           — commande vitesse (Twist)
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Packages ──────────────────────────────────────────────────────────────
    pkg_gazebo_ros        = get_package_share_directory('gazebo_ros')
    pkg_segway_description = get_package_share_directory('segway_description')
    pkg_segway_gazebo      = get_package_share_directory('segway_gazebo')

    # ── Arguments ─────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Utiliser le temps simulé Gazebo'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Lancer l\'interface graphique Gazebo'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_segway_gazebo, 'worlds', 'segway_flat.world'),
        description='Fichier world Gazebo'
    )
    # Hauteur spawn — légèrement au-dessus du sol, le robot tombera
    z_spawn_arg = DeclareLaunchArgument(
        'z_spawn', default_value='0.10',
        description='Hauteur de spawn du robot (m)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui          = LaunchConfiguration('gui')
    world        = LaunchConfiguration('world')
    z_spawn      = LaunchConfiguration('z_spawn')

    # ── URDF via xacro ────────────────────────────────────────────────────────
    xacro_file = os.path.join(
        pkg_segway_description, 'urdf', 'segway.urdf.xacro'
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── Noeuds ────────────────────────────────────────────────────────────────

    # 1. Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui':   gui,
        }.items()
    )

    # 2. robot_state_publisher — URDF → TF (frames fixes + roues)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # 3. Spawner — injecte le robot dans Gazebo
    #    z = 0.10m → robot spawn légèrement au-dessus, tombe sur ses roues
    #    puis bascule (pas de contrôleur à cette étape)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_segway',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'segway_mini',
            '-x', '0.0',
            '-y', '0.0',
            '-z', z_spawn,
            '-R', '0.0',
            '-P', '0.0',   # pitch=0 : robot vertical au spawn
            '-Y', '0.0',
        ],
        output='screen'
    )

    # ── LaunchDescription ─────────────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        world_arg,
        z_spawn_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
