import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_file_path = os.path.join(get_package_share_directory('src_demo'), 'config', "cam_params.yaml")

    usb_cam_node = Node(
        package = "usb_cam", 
        executable = "usb_cam_node_exe",
        output='log',
        parameters=[config_file_path],
    )

    return LaunchDescription([
        usb_cam_node,
    ])