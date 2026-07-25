from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
import os
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():

    launch_gazebo_arg = DeclareLaunchArgument(
        "launch_gazebo",
        default_value ="True",
        description="Whether to launch gazebo or not."
    )

    launch_controller_arg = DeclareLaunchArgument(
            "launch_controller",
            default_value ="True",
            description="Whether to launch controller or not."
        )

    launch_bridges_arg = DeclareLaunchArgument(
            "launch_bridges",
            default_value ="True",
            description="Whether to launch bridges or not, for bridging gz topics to ros."
        )

    launch_display_arg= DeclareLaunchArgument(
            "launch_rviz",
            default_value ="True",
            description="Whether to launch rviz or not."
        )

    launch_slam_arg = DeclareLaunchArgument(
            "launch_rtabmap",
            default_value ="True",
            description="Whether to launch rtabmap (SLAM) or not."
        )

    launch_nav_arg = DeclareLaunchArgument(
            "launch_nav2",
            default_value ="True",
            description="Whether to launch Nav2 or not."
        )

    launch_teleop_arg = DeclareLaunchArgument(
            "launch_teleop",
            default_value ="False",
            description="Whether to launch joystick teleop or not. Defaults to false"
        )
    
    launch_visualization_arg = DeclareLaunchArgument(
            "launch_vision",
            default_value ="False",
            description="Whether to launch some camera nodes and a yolo model or not. Defaults to false"
        )

    
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_description'), 
                        'launch', 'gazebo.launch.py')
        ), 
        condition=IfCondition(LaunchConfiguration('launch_gazebo'))
    )


    controllers = TimerAction(
        period=2.0,
        actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_controller'), 
                        'launch', 'controller.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("launch_controller"))
    )]
    )


    bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_bridges'),
                          'launch', 'bridge.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("launch_bridges"))
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_description'), 
                        'launch', 'display.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz"))
    )


    slam = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_slam'), 
                        'launch', 'slam.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_rtabmap'))
    )]
    )


    nav2 = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_nav'), 
                        'launch', 'nav.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("launch_nav2"))
    )]
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_controller'), 
                        'launch', 'joystick_teleop.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("launch_teleop"))
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fenrir_visualization'), 
                        'launch', 'visualization.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("launch_vision"))
    )



    return LaunchDescription([
        launch_gazebo_arg,
        launch_controller_arg,
        launch_bridges_arg,
        launch_display_arg,
        launch_slam_arg,
        launch_nav_arg,
        launch_teleop_arg,
        launch_visualization_arg,
        gazebo,
        controllers,
        bridges,
        rviz,
        slam,
        nav2,
        teleop,
        visualization
    ])