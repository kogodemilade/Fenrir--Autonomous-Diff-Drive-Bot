from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
import os
from ament_index_python import get_package_share_directory, get_package_prefix


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration('wheel_radius').perform(context))
    wheel_sep = float(LaunchConfiguration('wheel_seperation').perform(context))
    wheel_radius_err = float(LaunchConfiguration('wheel_radius_error').perform(context))
    wheel_sep_err = float(LaunchConfiguration('wheel_seperation_error').perform(context))
    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters=[{
            "wheel_radius": wheel_radius + wheel_radius_err,
            "wheel_seperation": wheel_sep + wheel_sep_err
        }]
    )
    return [noisy_controller_py]


def generate_launch_description():
    sim_time_parameter = SetParameter(name='use_sim_time', value=True)

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="True",
        description="Choose whether to use the python script or Cpp script."
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
        description="Radius of wheels"
    )

    wheel_sep_arg = DeclareLaunchArgument(
        "wheel_seperation",
        default_value="0.17",
        description="Distance between the center of both wheels."
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
        description="Use custom simple controller "
    )

    wheel_radius_err = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
        description="amount of error in error read"
    )

    wheel_sep_err = DeclareLaunchArgument(
        "wheel_seperation_error",
        default_value="0.01",
        description="Use custom simple controller "
    )

    
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_sep = LaunchConfiguration("wheel_seperation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]

    )

    ros_bridge = Node(
        package="ros_gz_bridge", 
        executable="parameter_bridge", 
        output="screen",
        arguments=["/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        parameters=[{'use_sim_time': True}])

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['simple_velocity_controller', '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': True}]
            ),

            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius": wheel_radius, "wheel_sep": wheel_sep}, {'use_sim_time': True}]
            )
        ]
    )

    # noisy_controller_spawner = OpaqueFunction(function=noisy_controller)
    noisy_controller_ = GroupAction(
        condition = UnlessCondition(use_simple_controller),
        actions=[OpaqueFunction(function=noisy_controller)]
        )

    ld = LaunchDescription()
    ld.add_action(sim_time_parameter)
    ld.add_action(use_simple_controller_arg)
    ld.add_action(use_python_arg)
    ld.add_action(wheel_radius_arg)
    ld.add_action(wheel_sep_arg)
    ld.add_action(wheel_radius_err)
    ld.add_action(wheel_sep_err)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(simple_controller)
    ld.add_action(noisy_controller_)
    ld.add_action(ros_bridge)
    return ld

