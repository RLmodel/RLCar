import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Create the launch configuration variables
    open_rviz = LaunchConfiguration('open_rviz')

    declare_open_rviz_cmd = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Launch Rviz?'
    )

    # Rplidar Driver
    rplidar_ros2_pkg = os.path.join(get_package_share_directory('rplidar_ros2'))
    src_odometry_pkg = os.path.join(get_package_share_directory('src_odometry'))
    src_slam_pkg = os.path.join(get_package_share_directory('src_slam'))
    
    serial_port = "/dev/rplidar"
    # rplidar_launch_file = "rplidar_a3_launch.py"
    rplidar_launch_file = "rplidar_launch.py"
    rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rplidar_ros2_pkg, 'launch', rplidar_launch_file)),
        launch_arguments={'serial_port': serial_port}.items()
    )

    src_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(src_odometry_pkg, 'launch', 'src_bringup.launch.py')),
    )

    # x y z yaw pitch roll frame_id child_frame_id period_in_ms
    static_transform_publisher = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments=["0.24", "0", "0.14", "3.1415", "0", "0", "base_link", "laser"]
    )

    # Rviz
    rviz_config_dir = os.path.join(src_slam_pkg, 'rviz', 'rplidar_view.rviz')

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

    slam_params_file = os.path.join(src_slam_pkg, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(src_slam_pkg, 'rviz', 'slam_toolbox_real.rviz')

    slam_toolbox_with_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(src_slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'rviz_config_file': rviz_config_file,
            'open_rviz': open_rviz,
        }.items()
    )

    return LaunchDescription([
        declare_open_rviz_cmd,
        
        rplidar_driver,
        src_bringup,
        static_transform_publisher,

        # rf2o_laser_odometry,
        slam_toolbox_with_rviz,
    ])