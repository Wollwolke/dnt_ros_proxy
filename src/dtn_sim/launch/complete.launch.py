from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_file_dir = PathJoinSubstitution([FindPackageShare("dtn_sim"), "launch"])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dtn_sim"), "rviz", "dtn_sim_default.rviz"]
    )

    start_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, "empty_world.launch.py"])
        )
    )

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, "navigation2.launch.py"])
        )
    )

    start_transform_pub_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            start_nav2_cmd,
            start_sim_cmd,
            start_rviz_cmd,
            start_transform_pub_cmd,
        ]
    )
