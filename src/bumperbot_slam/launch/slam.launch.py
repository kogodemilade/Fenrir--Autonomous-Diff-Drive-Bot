from launch import LaunchDescription
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer
from launch.actions import LogInfo, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Whether to use sim time or not (LEAVE AS TRUE YOU TWAT)')

    sim_time_parameter = SetParameter(name='use_sim_time', value=True)

    pkg_share = get_package_share_directory('bumperbot_slam')
    bridge_config_path = os.path.join(pkg_share, 'config', 'slam_config.yaml')
    rtabmap_config_path = os.path.join(pkg_share, 'config', 'rtabmap.yaml')


    ros_gz_br = RosGzBridge(
        bridge_name='slam_bridge',
        config_file=bridge_config_path,
        # extra_bridge_params=[{'placeholder': LaunchConfiguration('use_sim_time')}]
    )

    # rtabmap = Node(
    #         package='rtabmap_slam',
    #         executable='rtabmap',
    #         name='rtabmap',
    #         parameters=[rtabmap_config_path],
    #         arguments=['--delete_db_on_start'],
    #         remappings=[
    #             ('rgb/image', '/camera1/image/image'),
    #             ('depth/image', '/camera1/image/depth_image'),
    #             ('odom', '/odom'),
    #             ('rgb/camera_info', '/camera1/image/camera_info'),
    #             ('scan_cloud', '/lidar/cloud/points')
    #         ]
    #     )

    rgbd_sync_container = ComposableNodeContainer(
        name='rgbd_sync_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multi-threaded
        # parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        composable_node_descriptions=[
            ComposableNode(
                package='rtabmap_sync',
                plugin='rtabmap_sync::RGBDSync',
                name='rgbd_sync',
                remappings=[
                    ('rgb/image', '/camera1/image'),
                    ('depth/image', '/camera1/depth_image'),
                    ('rgb/camera_info', '/camera1/camera_info'),
                    ('rgbd_image', 'rgbd_image')  # output
                ],
            )
        ],
        output='screen'
    )

    # RTAB-Map SLAM Node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[rtabmap_config_path],  
        arguments=['--delete_db_on_start'],
        remappings=[
            ('rgb/image', 'rgbd_image/rgb/image'),          # from sync node
            ('depth/image', 'rgbd_image/depth/image'),      # from sync node
            ('rgb/camera_info', 'rgbd_image/rgb/camera_info'),
            ('odom', '/bumperbot_controller/odom'),
            ('scan', '/lidar/scan')
        ],
        output='screen'
    )

    diffchecker = Node(
        package='bumperbot_slam',
        executable='diff_checker',
        name='diff_node',
        output="screen"
    )
    



    log_config_path = LogInfo(
        condition=None,
        msg=bridge_config_path
    )

    return LaunchDescription([
        sim_time_arg,
        sim_time_parameter,
        rgbd_sync_container,
        log_config_path,
        ros_gz_br,
        rtabmap,
        # diffchecker
    ])
