from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch the FR3 Gazebo/MoveIt stack with RLC demo components enabled.

    Returns
    -------
    launch.LaunchDescription
        Launch description that wraps `fr3_gz_moveit.launch.py`.
    """
    default_tree = LaunchConfiguration("default_tree")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "default_tree",
                default_value="record_servo_demos.xml",
                description="Default BehaviorTree XML path or filename.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rlc_bringup"),
                            "launch",
                            "fr3_gz_moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "launch_executive": "true",
                    "launch_demo": "true",
                    "default_tree": default_tree,
                }.items(),
            ),
        ]
    )
