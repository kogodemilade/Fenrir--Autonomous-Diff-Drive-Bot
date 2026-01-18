#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    stamped_arg = DeclareLaunchArgument(
        'stamped', default_value='true',
        description='Publish geometry_msgs/TwistStamped instead of Twist'
    )
    frame_arg = DeclareLaunchArgument(
        'frame_id', default_value='base_link',
        description='frame_id to stamp into TwistStamped header'
    )
    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/bumperbot_controller/cmd_vel',
        description='Target cmd_vel ROS topic for your controller'
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{
            'stamped': LaunchConfiguration('stamped'),
            'frame_id': LaunchConfiguration('frame_id')
        }],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
        prefix="xterm -e"
    )

    return LaunchDescription([
        stamped_arg,
        frame_arg,
        cmd_vel_arg,
        teleop_node
    ])
