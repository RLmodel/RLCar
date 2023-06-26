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

    # Rviz
    src_slam_pkg = os.path.join(get_package_share_directory('src_slam'))
    rviz_config_file = os.path.join(src_slam_pkg, 'rviz', 'slam_toolbox_real.rviz')

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )

    return LaunchDescription([
        rviz2,
    ])