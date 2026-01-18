from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import SetParameter
import os
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    sim_time_parameter = SetParameter(name='use_sim_time', value=True)
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("bumperbot_description"), "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to URDF file"
        )
    

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_log = LogInfo(
        msg=f'robot_description: {robot_description}'
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        # parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("bumperbot_description"), "rviz", "display.rviz")],
        # parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    return LaunchDescription(
        [
        sim_time_parameter,
        model_arg, 
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
        robot_log
        ]
    )