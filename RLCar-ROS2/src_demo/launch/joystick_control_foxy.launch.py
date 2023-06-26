import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joy_to_cmd_vel = Node(
        package='joy_to_cmd',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen'
    )

    cmd_to_src_pkg = os.path.join(get_package_share_directory('cmd_to_src'))
    cmd_to_src = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cmd_to_src_pkg, 'launch', 'cmd_to_src.launch.py')),
    )

    return LaunchDescription([
        joy,
        joy_to_cmd_vel,
        cmd_to_src,
    ])