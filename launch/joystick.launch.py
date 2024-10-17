from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('differential_robot'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name="teleop_node",  # The default node name for teleop_node executable is teleop_twist_joy_node. Keeping it as-is is not a problem but it can be a bit confusing so we changed it to teleop_node.
        parameters=[joy_params],  # passing same params file as the one passed for the joy_node
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])