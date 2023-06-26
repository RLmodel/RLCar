import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Rviz
    mw_ahrsv1_ros2_pkg = os.path.join(get_package_share_directory('mw_ahrsv1_ros2'))
    rviz_config_dir = os.path.join(mw_ahrsv1_ros2_pkg, 'rviz', 'view_imu.rviz')

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )

    mw_ahrs_node = Node(
        package='mw_ahrsv1_ros2',
        executable='mw_ahrsv1_ros2',
        name='mw_ahrsv1_ros2',
        output='log',
        parameters=[{
            'deviceID' : '/dev/MWAHRs',
            'publish_tf' : False,
            'verbose' : False,
            'publish_rate' : 50
        }],
    )

    return LaunchDescription([
        mw_ahrs_node,
        # rviz2,
    ])