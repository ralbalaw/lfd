from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch the MoveIt-to-Tesseract scene bridge node.

    Returns
    -------
    launch.LaunchDescription
        Launch description for `rlc_scene_bridge_node`.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    config = LaunchConfiguration("config")
    monitor_namespace = LaunchConfiguration("monitor_namespace")
    env_name = LaunchConfiguration("env_name")
    scene_topic = LaunchConfiguration("scene_topic")
    get_scene_srv = LaunchConfiguration("get_scene_srv")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the scene bridge node.",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rlc_scene_bridge"),
                        "config",
                        "scene_bridge.yaml",
                    ]
                ),
                description="ROS parameter YAML for the scene bridge node.",
            ),
            DeclareLaunchArgument(
                "monitor_namespace",
                default_value="tesseract_monitor",
                description="Tesseract monitor namespace to synchronize.",
            ),
            DeclareLaunchArgument(
                "env_name",
                default_value="fr3",
                description="Tesseract environment name used by monitor clients.",
            ),
            DeclareLaunchArgument(
                "scene_topic",
                default_value="/monitored_planning_scene",
                description="MoveIt planning-scene topic consumed by the bridge.",
            ),
            DeclareLaunchArgument(
                "get_scene_srv",
                default_value="/get_planning_scene",
                description="MoveIt GetPlanningScene service consumed by the bridge.",
            ),
            Node(
                package="rlc_scene_bridge",
                executable="rlc_scene_bridge_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "scene_bridge.monitor_namespace": monitor_namespace,
                        "scene_bridge.env_name": env_name,
                        "scene_bridge.scene_topic": scene_topic,
                        "scene_bridge.get_scene_srv": get_scene_srv,
                    },
                ],
            ),
        ]
    )
