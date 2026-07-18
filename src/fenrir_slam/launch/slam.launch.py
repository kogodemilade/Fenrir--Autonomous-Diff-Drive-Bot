from launch import LaunchDescription
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use sim time or not (LEAVE AS TRUE YOU TWAT)'
    )

    sim_time_parameter = SetParameter(name='use_sim_time', value=True)

    pkg_share = get_package_share_directory('fenrir_slam')
    bridge_config_path = os.path.join(pkg_share, 'config', 'slam_config.yaml')
    rtabmap_config_path = os.path.join(pkg_share, 'config', 'rtabmap.yaml')
    pcloud_to_ls_path = os.path.join(pkg_share, 'config', 'pointcloud_to_ls.yaml')

    ros_gz_br = RosGzBridge(
        bridge_name='slam_bridge',
        config_file=bridge_config_path,
    )

    rgbd_sync_container = ComposableNodeContainer(
        name='rgbd_sync_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        composable_node_descriptions=[

            ComposableNode(
                package='rtabmap_sync',
                plugin='rtabmap_sync::RGBDSync',
                name='rgbd_sync',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'approx_sync': False,
                }],
                remappings=[
                    ('rgb/image',       '/camera1/image'),
                    ('depth/image',     '/camera1/depth_image'),
                    ('rgb/camera_info', '/camera1/camera_info'),
                    ('rgbd_image',      '/rgbd_image'),
                ],
            ),

            ComposableNode(
                package='rtabmap_slam',
                plugin='rtabmap_slam::CoreWrapper',
                name='rtabmap',
                parameters=[
                    rtabmap_config_path,
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'delete_db_on_start': True},
                ],
                remappings=[
                    ('rgbd_image', '/rgbd_image'),
                    ('odom',       '/fenrir_controller/odom'),
                    ('scan_cloud',       '/lidar/cloud/points'),
                ],
            ),

        ],
        output='screen'
    )

    pcloud_to_ls = Node(
    package="pointcloud_to_laserscan",
    executable="pointcloud_to_laserscan_node",
    name="pointcloud_to_laserscan",
    parameters=[pcloud_to_ls_path],
    remappings=[
        ("cloud_in", "/lidar/cloud/points"),
        ("scan", "/scan"),
    ],
)

    # log_config_path = LogInfo(msg=bridge_config_path)

    return LaunchDescription([
        sim_time_arg,
        sim_time_parameter,
        # log_config_path,
        # ros_gz_br,
        rgbd_sync_container,
        pcloud_to_ls
    ])