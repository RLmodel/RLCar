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

    # Rplidar Driver
    rplidar_ros2_pkg = os.path.join(get_package_share_directory('rplidar_ros2'))
    serial_port = "/dev/rplidar"

    rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rplidar_ros2_pkg, 'launch', 'rplidar_a3_launch.py')),
        launch_arguments={'serial_port': serial_port}.items()
    )

    # Rviz
    src_slam_pkg = os.path.join(get_package_share_directory('src_slam'))
    rviz_config_dir = os.path.join(src_slam_pkg, 'rviz', 'rplidar_view.rviz')

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )

    rf2o_laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : True,
            'base_frame_id' : 'laser',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0}],
    )

    slam_params_file = os.path.join(src_slam_pkg, 'config', 'mapper_params_online_async_laser_only.yaml')
    slam_toolbox_with_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(src_slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    return LaunchDescription([
        rplidar_driver,
        # rviz2,
        rf2o_laser_odometry,
        slam_toolbox_with_rviz,
    ])