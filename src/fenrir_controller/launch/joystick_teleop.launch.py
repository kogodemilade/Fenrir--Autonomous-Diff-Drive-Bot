from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("fenrir_controller"), "config", "joy_config.yaml")
    teleop_path = os.path.join(get_package_share_directory("fenrir_controller"), "config", "joy_teleop.yaml")
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[config_path],
    )

    joy_teleop = Node(
    package="teleop_twist_joy",
    executable="teleop_node",
    name="teleop_twist_joy",
    parameters=[teleop_path],
    remappings=[
        ("cmd_vel", "/fenrir_controller/cmd_vel"),
    ],
)

    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(joy_teleop)

    return ld