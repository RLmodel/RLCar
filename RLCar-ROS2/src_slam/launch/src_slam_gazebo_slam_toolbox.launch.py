import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from osrf_pycommon.terminal_color import ansi


def generate_launch_description():

    src_slam_pkg = os.path.join(get_package_share_directory('src_slam'))

    slam_params_file = os.path.join(src_slam_pkg, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(src_slam_pkg, 'rviz', 'slam_toolbox_gazebo.rviz')

    slam_toolbox_with_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(src_slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'rviz_config_file': rviz_config_file,
        }.items()
    )

    return LaunchDescription([
        slam_toolbox_with_rviz,
    ])