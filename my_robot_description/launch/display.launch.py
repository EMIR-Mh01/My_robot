from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Chemin du fichier xacro
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'robot.urdf.xacro'
    )

    # Traitement du fichier xacro
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    # Paramètres communs
    sim_time_param = {'use_sim_time': True}

    return LaunchDescription([
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[sim_time_param]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description_config},
                sim_time_param
            ]
        ),
    ])

