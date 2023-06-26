import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("test_description"))
    rviz_config_file = os.path.join(pkg_path, "rviz", "description.rviz")
    xacro_file = os.path.join(pkg_path, "urdf", "src_description.urdf.xacro")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "prefix:=",
            '""',
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Launch RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        joint_state_publisher, 
        robot_state_publisher, 
        joint_state_publisher_gui, 
        rviz
    ])  
