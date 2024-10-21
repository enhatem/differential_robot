from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('differential_robot'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name="teleop_node",  # The default node name for teleop_node executable is teleop_twist_joy_node. Keeping it as-is is not a problem but it can be a bit confusing so we changed it to teleop_node.
        parameters=[joy_params, {'use_sim_time': use_sim_time}],  # passing same params file as the one passed for the joy_node
        # remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
        remappings=[('/cmd_vel', '/cmd_vel_joy')]  # for the twist_mux node
    )

    # twist_stamper = Node(
    #     package='twist_stamper',
    #     executable='twist_stamper',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[('/cmd_vel_in', '/diff_cont/cmd_vel_unstamped'),
    #                 ('/cmd_vel_out', '/diff_cont/cmd_vel')]
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        # twist_stamper
    ])