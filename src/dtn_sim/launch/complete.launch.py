from ament_index_python.packages import get_package_share_directory
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

    start_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, "empty_world.launch.py"])
        )
    )

    return LaunchDescription([start_sim_cmd])
