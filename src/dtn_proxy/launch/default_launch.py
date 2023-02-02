from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    # mode_launch_arg = DeclareLaunchArgument(
    #     "stationMode", default_value=TextSubstitution(text="false")
    # )
    log_lvl_launch_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("DEBUG")),
        description="Logging level",
    )

    dtnproxy_node_bot = Node(
        package="dtn_proxy",
        executable="dtnproxy",
        name="dtn1",
        output="screen",
        # parameters=[{"stationMode": LaunchConfiguration("stationMode")}],
        arguments=[
            "--ros-args",
            # ! breaks when using namespaces...
            "--log-level",
            [
                "dtn1:=",
                LaunchConfiguration("log_level"),
            ],
        ],
    )

    return LaunchDescription([log_lvl_launch_arg, dtnproxy_node_bot])  # mode_launch_arg