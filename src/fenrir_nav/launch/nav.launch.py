from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('fenrir_nav')
    params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            output='screen',
            parameters=[params],
        )
    ])
