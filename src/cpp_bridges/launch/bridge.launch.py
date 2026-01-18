from launch import LaunchDescription
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import SetParameter, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Whether to use sim time or not (LEAVE AS TRUE YOU TWAT)')

    sim_time_parameter = SetParameter(name='use_sim_time', value=True)

    pkg_share = get_package_share_directory('bumperbot_bridges')
    config_path = os.path.join(pkg_share, 'config', 'bridge.yaml')

    ros_gz_br = RosGzBridge(
        bridge_name='camera_bridge',
        config_file=config_path,
        extra_bridge_params=[{'placeholder': LaunchConfiguration('use_sim_time')}],
    )

    camera_stream = Node(        
        package="bumperbot_bridges",
        executable="camera_stream",
        name="camera_stream",
        output="screen",
        additional_env={
        'PYTHONPATH':
            [EnvironmentVariable('HOME'),
             TextSubstitution(text='/bumperbot_ws/.venv/lib/python3.12/site-packages:'),
             EnvironmentVariable('PYTHONPATH')]
    }
    )

    ld = LaunchDescription() 
    ld.add_action(sim_time_arg)
    ld.add_action(sim_time_parameter)
    ld.add_action(ros_gz_br)
    ld.add_action(camera_stream)
    return ld