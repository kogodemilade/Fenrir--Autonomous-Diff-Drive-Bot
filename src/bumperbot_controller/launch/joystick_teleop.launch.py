from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_config.yaml")
    teleop_path = os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_teleop.yaml")
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[config_path],
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[teleop_path]
    )

    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(joy_teleop)

    return ld