from launch import LaunchDescription
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node, SetParameter
from launch.actions import LogInfo, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Whether to use sim time or not (LEAVE AS TRUE YOU TWAT)')

    sim_time_parameter = SetParameter(name='use_sim_time', value=True)

    pkg_share = get_package_share_directory('bumperbot_visualization')
    config_path = os.path.join(pkg_share, 'config', 'image_bridge.yaml')

    # set_env_vars = SetEnvironmentVariable(name='PYTHONPATH', 
    #                                       value="$HOME/bumperbot_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH")
    # Bridge camera topics

    camera_stream = Node(        
        package="bumperbot_visualization",
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

    vision = Node(        
        package="bumperbot_visualization",
        executable="vision",
        name="vision_node",
        output="screen",
        additional_env={
        'PYTHONPATH':
            [EnvironmentVariable('HOME'),
             TextSubstitution(text='/bumperbot_ws/.venv/lib/python3.12/site-packages:'),
             EnvironmentVariable('PYTHONPATH')]
    }
    )

    log_config_path = LogInfo(
        condition=None,
        msg=config_path
    )

    return LaunchDescription([
        sim_time_arg,
        sim_time_parameter,
        log_config_path,
        vision
    ])
