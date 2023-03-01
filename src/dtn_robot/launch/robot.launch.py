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

    control_interface_cmd = Node(
        package="dtn_robot",
        executable="control_interface",
        name="controlInterface",
        output="screen",
    )

    temp_fake_cmd = Node(
        package="dtn_robot",
        executable="temp_sensor",
        name="tempSensorFake",
        output="screen",
    )

    return LaunchDescription(
        [
            temp_fake_cmd,
            control_interface_cmd,
        ]
    )
