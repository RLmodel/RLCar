#!/usr/bin/env python3
#
# Copyright 2022 RoadBalance Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from osrf_pycommon.terminal_color import ansi

import xacro

def generate_launch_description():

    gazebo_model_path = os.path.join(get_package_share_directory('src_gazebo'), 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    src_gazebo_path = os.path.join(get_package_share_directory('src_gazebo'))
    pkg_path = os.path.join(get_package_share_directory('src_sensor_fusion'))

    # launch configuration
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Start Gazebo server
    world_path = os.path.join(src_gazebo_path, 'worlds', 'racecar_course.world')
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher
    src_gazebo_path = os.path.join(get_package_share_directory('src_gazebo'))
    urdf_file = os.path.join(src_gazebo_path, 'urdf', 'src_ackermann.urdf')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn Robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'racecar'],
                        output='screen')

    # ROS 2 controller
    load_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
        output="screen",
    )

    load_velocity_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller"],
        output="screen",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Racecar controller launch
    src_gazebo_controller = Node(
        package='src_gazebo_controller',
        executable='src_gazebo_controller',
        output='screen',
        parameters=[
            {
                "verbose": False,
            }
        ],
    )

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'wheel_lidar_odom.rviz')

    # Launch RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    src_odometry = Node(
        package='src_odometry',
        executable='src_odometry',
        name='src_odometry',
        output='log',
        parameters=[{
            "verbose" : False,
            'publish_rate' : 50,
            'open_loop' : False,
            'has_imu_heading' : True,
            'is_gazebo' : True,
            'wheel_radius' : 0.0508,
            'base_frame_id' : "base_footprint",
            'odom_frame_id' : "odom",
            'enable_odom_tf' : False,
        }],
    )

    rf2o_laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0}],
    )

    # rqt robot steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    return LaunchDescription([
        declare_use_rviz,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_forward_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_forward_position_controller,
                on_exit=[load_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[src_gazebo_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[src_odometry],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_velocity_controller,
        #         on_exit=[rviz],
        #     )
        # ),

        TimerAction(    
            period=7.0,
            actions=[rviz]
        ),

        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        rf2o_laser_odometry,
        rqt_robot_steering,
    ])