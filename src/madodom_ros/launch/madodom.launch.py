import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("madodom_ros")

    rviz_config = PathJoinSubstitution([pkg_share, "config", "madodom.rviz"])
    params_file = PathJoinSubstitution([pkg_share, "config", "madodom.yaml"])

    use_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz2 to visualise the odometry",
    )

    madodom_node = Node(
        package="madodom_ros",
        executable="madodom_node",
        name="madodom",
        parameters=[params_file],
        output="screen",
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        use_rviz_arg,
        madodom_node,
        rviz_node,
    ])
