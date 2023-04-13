from launch import LaunchDescription
from launch.substitutions import (
    TextSubstitution,
    LaunchConfiguration,
)
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    log_lvl_launch_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("INFO")),
        description="Logging level",
    )

    control_interface_cmd = Node(
        package="dtn_robot",
        executable="control_interface",
        name="controlInterface",
        output="screen",
    )

    status_publisher_cmd = Node(
        package="dtn_robot",
        executable="status_pub",
        name="statusPublisher",
        output="screen",
    )

    dht_fake_cmd = Node(
        package="dtn_robot",
        executable="dht_fake",
        name="dhtSensorFake",
        output="screen",
        arguments=[
            "--ros-args",
            # ! breaks when using namespaces...
            "--log-level",
            [
                "dhtSensorFake:=",
                LaunchConfiguration("log_level"),
            ],
        ],
    )

    image_publisher_cmd = Node(
        package="dtn_robot",
        executable="image_pub",
        name="imagePublisher",
        output="screen",
    )

    return LaunchDescription(
        [
        log_lvl_launch_arg,
        dht_fake_cmd,
        control_interface_cmd,
        status_publisher_cmd,
        image_publisher_cmd
        ]
    )
