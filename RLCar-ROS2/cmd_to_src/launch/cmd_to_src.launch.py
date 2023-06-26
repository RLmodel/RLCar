import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cmd_to_src = Node(
        package='cmd_to_src',
        executable='cmd_to_src',
        name='cmd_to_src',
        output='screen',
        parameters=[{
            "accel_scale" : 0.5,
            "deaccel_scale" : 0.5,
            "scale" : 40, # linear 하지 못하다. 0.1에서는 좀 못미치고, 0.4에서는 넘어버린다.
            "p_gain" : 80.0, # 95.0
            "i_gain" : 2.0,
            "d_gain" : 0.5,
            "steering_offset" : 28,
            "use_twiddle" : False,
        }],
    )

    return LaunchDescription([
        cmd_to_src
    ])
