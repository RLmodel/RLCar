import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Set the path to different files and folders.
    pkg_path = os.path.join(get_package_share_directory('src_sensor_fusion'))

    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = os.path.join(pkg_path, 'config', 'imu_ekf.yaml') 
    
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            robot_localization_file_path, 
            {'use_sim_time': True}
        ],
        # remappings=[("odometry/filtered", "odom")],
    )

    return LaunchDescription([
        robot_localization,
    ])