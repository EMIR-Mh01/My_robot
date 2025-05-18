import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'joystick.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        name='joy_node',
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        output='screen'
    )

    # Optional: twist_stamper node if you want to stamp cmd_vel before sending it to the controller
    # twist_stamper = Node(
    #     package='twist_stamper',
    #     executable='twist_stamper',
    #     name='twist_stamper',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[
    #         ('/cmd_vel_in', '/cmd_vel_joy'),
    #         ('/cmd_vel_out', '/diff_cont/cmd_vel')
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        joy_node,
        teleop_node,
        # twist_stamper
    ])
