from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Récupérer les chemins nécessaires
    pkg_description = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_description, 'worlds', 'world_test.world')
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'robot.rviz')

    # Paramètres
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}
    sim_time_param = {'use_sim_time': True}

    return LaunchDescription([
        # Lancer Gazebo avec le monde personnalisé
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Publier les états des joints (obligatoire pour RViz)
        Node(
            package='joint_state_publisher',  # Change 'joint_state_publisher_gui' si tu veux GUI
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[sim_time_param]
        ),

        # Publier les TF à partir de robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, sim_time_param]
        ),

        # Spawner du robot dans Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
            output='screen',
            parameters=[sim_time_param]
        ),

        # Lancer RViz avec temporisation
        TimerAction(
            period=6.0,  # délai pour que les TF soient prêts
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_file],
                    parameters=[sim_time_param]
                )
            ]
        )
    ])

