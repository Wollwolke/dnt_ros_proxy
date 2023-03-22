from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Select Turtlebot model (with / without sensors)
# TURTLEBOT3_MODEL = "waffle_pi"
TURTLEBOT3_MODEL = "pi_camera_dtn"
# TURTLEBOT3_MODEL = "pi_dtn"


def generate_launch_description():
    ld = LaunchDescription()
    launch_file_dir = PathJoinSubstitution([FindPackageShare("dtn_sim"), "launch"])

    if TURTLEBOT3_MODEL != "waffle_pi":
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("dtn_sim"), "rviz", "dtn_sim_default.rviz"]
        )

        start_transform_pub_cmd = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        )
        ld.add_action(start_transform_pub_cmd)

        if TURTLEBOT3_MODEL == "pi_dtn":
            world_file = "empty_world.world"
        else:
            world_file = "RoboCup2019_final.world"
    else:
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("dtn_sim"), "rviz", "dtn_sim_waffle_pi.rviz"]
        )
        world_file = "RoboCup2019_final.world"

    set_env_action = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL", value=TURTLEBOT3_MODEL
    )

    start_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, "empty_world.launch.py"])
        ),
        launch_arguments={"world_file": world_file}.items(),
    )

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, "navigation2.launch.py"])
        )
    )

    start_virtual_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([PathJoinSubstitution([FindPackageShare("dtn_robot"), "launch"]), "robot.launch.py"])
        )
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    ld.add_action(set_env_action)
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_sim_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_virtual_robot_cmd)

    return ld
