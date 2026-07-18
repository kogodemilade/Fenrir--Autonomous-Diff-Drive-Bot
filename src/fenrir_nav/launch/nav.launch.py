from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('fenrir_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    controller = Node(
    package="nav2_controller",
    executable="controller_server",
    parameters=[params_file],
    )

    planner = Node(
    package="nav2_planner",
    executable="planner_server",
    parameters=[params_file],
    )

    behaviours = Node(
    package="nav2_behaviors",
    executable="behavior_server",
    parameters=[params_file],
    )   

    bt_navigator = Node(
    package="nav2_bt_navigator",
    executable="bt_navigator",
    parameters=[params_file],
    )

    manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    parameters=[{
        "autostart": True,
        "node_names": [
            "controller_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
        ]
    }],
)
    ld = LaunchDescription()
    ld.add_action(controller)
    ld.add_action(planner)
    ld.add_action(behaviours)
    ld.add_action(bt_navigator)
    ld.add_action(manager)

    return ld
    
