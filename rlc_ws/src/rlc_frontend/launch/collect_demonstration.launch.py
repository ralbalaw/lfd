from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch the frontend demonstration collection action server.

    Returns
    -------
    launch.LaunchDescription
        Launch description for the `collect_demonstration_server` node.
    """
    config = LaunchConfiguration("config")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rlc_frontend"),
                        "config",
                        "collect_demonstration.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true.",
            ),
            Node(
                package="rlc_frontend",
                executable="collect_demonstration_server",
                name="collect_demonstration_server",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    config,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
        ]
    )
