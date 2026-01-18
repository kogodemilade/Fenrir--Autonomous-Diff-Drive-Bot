from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get the path to bridge.yaml
    bridge_config = PathJoinSubstitution([
        FindPackageShare('bumperbot_bridges'),
        'config',
        'bridge.yaml'
    ])
    
    # Ros GZ Bridge node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        parameters=[
            {'config_file': bridge_config},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Camera stream node
    camera_node = Node(
        package='bumperbot_bridges',
        executable='camera_stream',
        name='camera_stream',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        bridge_node,
        camera_node
    ])