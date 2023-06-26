import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy = Node(
        package='joy',
        node_executable='joy_node',
        node_name='joy_node',
        output='screen'
    )

    joy_to_cmd_vel = Node(
        package='joy_to_cmd',
        node_executable='joy_to_cmd_vel',
        node_name='joy_to_cmd_vel',
        output='screen'
    )

    cmd_to_src = Node(
        package='cmd_to_src',
        node_executable='cmd_to_src',
        node_name='cmd_to_src',
        output='screen'
    )

    return LaunchDescription([
        joy,
        joy_to_cmd_vel,
        cmd_to_src,
    ])