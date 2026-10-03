import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_TESSERACT_XMLNS = "https://github.com/tesseract-robotics/tesseract"


def _ensure_tesseract_make_convex_attribute(
    urdf_xml: str, *, make_convex: bool = True
) -> str:
    """
    Add Tesseract convex-mesh attributes to the URDF robot tag if missing.

    Parameters
    ----------
    urdf_xml : str
        URDF XML text.
    make_convex : bool, optional
        Value to use for `tesseract:make_convex`.

    Returns
    -------
    str
        URDF XML text with Tesseract attributes present.

    Raises
    ------
    RuntimeError
        If the URDF does not contain a complete `<robot ...>` start tag.
    """
    robot_open_idx = urdf_xml.find("<robot")
    if robot_open_idx < 0:
        raise RuntimeError("URDF XML does not contain a '<robot' element.")

    robot_tag_end_idx = urdf_xml.find(">", robot_open_idx)
    if robot_tag_end_idx < 0:
        raise RuntimeError(
            "URDF XML contains '<robot' but no closing '>' for the start tag."
        )

    robot_start_tag = urdf_xml[robot_open_idx:robot_tag_end_idx]
    insert = ""

    if "xmlns:tesseract=" not in robot_start_tag:
        insert += f' xmlns:tesseract="{_TESSERACT_XMLNS}"'

    if "tesseract:make_convex=" not in robot_start_tag:
        insert += f' tesseract:make_convex="{"true" if make_convex else "false"}"'

    if not insert:
        return urdf_xml

    return urdf_xml[:robot_tag_end_idx] + insert + urdf_xml[robot_tag_end_idx:]


def generate_launch_description() -> LaunchDescription:
    """
    Launch Tesseract environment monitoring and MoveIt scene synchronization.

    Returns
    -------
    launch.LaunchDescription
        Launch description for the Tesseract monitor and scene bridge.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    urdf_xacro_file = LaunchConfiguration("urdf_xacro_file")
    urdf_xacro_args = LaunchConfiguration("urdf_xacro_args")
    srdf_xacro_file = LaunchConfiguration("srdf_xacro_file")
    srdf_xacro_args = LaunchConfiguration("srdf_xacro_args")
    monitor_namespace = LaunchConfiguration("monitor_namespace")
    env_name = LaunchConfiguration("env_name")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    monitored_namespace = LaunchConfiguration("monitored_namespace")
    bridge_config = LaunchConfiguration("bridge_config")
    scene_topic = LaunchConfiguration("scene_topic")
    get_scene_srv = LaunchConfiguration("get_scene_srv")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for Tesseract scene-sync nodes.",
        ),
        DeclareLaunchArgument(
            "urdf_xacro_file",
            default_value="",
            description="URDF xacro file used to generate `robot_description`.",
        ),
        DeclareLaunchArgument(
            "urdf_xacro_args",
            default_value="",
            description="Extra xacro args appended to `urdf_xacro_file`.",
        ),
        DeclareLaunchArgument(
            "srdf_xacro_file",
            default_value="",
            description="SRDF xacro file used to generate `robot_description_semantic`.",
        ),
        DeclareLaunchArgument(
            "srdf_xacro_args",
            default_value="",
            description="Extra xacro args appended to `srdf_xacro_file`.",
        ),
        DeclareLaunchArgument(
            "monitor_namespace",
            default_value="tesseract_monitor",
            description="Tesseract monitor namespace used for services/topics.",
        ),
        DeclareLaunchArgument(
            "env_name",
            default_value="fr3",
            description="Tesseract environment name used by monitor clients.",
        ),
        DeclareLaunchArgument(
            "joint_state_topic",
            default_value="",
            description="Joint state topic passed to the Tesseract monitor.",
        ),
        DeclareLaunchArgument(
            "monitored_namespace",
            default_value="",
            description="MoveIt namespace monitored by the Tesseract environment node.",
        ),
        DeclareLaunchArgument(
            "bridge_config",
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
            "scene_topic",
            default_value="/monitored_planning_scene",
            description="MoveIt planning-scene topic consumed by the bridge.",
        ),
        DeclareLaunchArgument(
            "get_scene_srv",
            default_value="/get_planning_scene",
            description="MoveIt GetPlanningScene service consumed by the bridge.",
        ),
    ]

    def launch_setup(context, *args, **kwargs):  # noqa: ANN001, ARG001
        """
        Create scene-sync actions after resolving xacro inputs.

        Parameters
        ----------
        context : launch.LaunchContext
            Launch context used to evaluate substitutions.
        *args
            Unused positional arguments required by `OpaqueFunction`.
        **kwargs
            Unused keyword arguments required by `OpaqueFunction`.

        Returns
        -------
        list[launch.Action]
            Tesseract monitor and scene bridge actions.

        Raises
        ------
        RuntimeError
            If required xacro inputs are missing or invalid.
        """
        urdf_xacro_file_value = urdf_xacro_file.perform(context).strip()
        srdf_xacro_file_value = srdf_xacro_file.perform(context).strip()
        if not urdf_xacro_file_value or not srdf_xacro_file_value:
            raise RuntimeError(
                "Both `urdf_xacro_file` and `srdf_xacro_file` must be provided."
            )
        if not os.path.exists(urdf_xacro_file_value):
            raise RuntimeError(f"URDF xacro file not found: '{urdf_xacro_file_value}'")
        if not os.path.exists(srdf_xacro_file_value):
            raise RuntimeError(f"SRDF xacro file not found: '{srdf_xacro_file_value}'")

        urdf_xacro_args_value = urdf_xacro_args.perform(context).strip()
        srdf_xacro_args_value = srdf_xacro_args.perform(context).strip()

        robot_description_cmd = [
            FindExecutable(name="xacro"),
            " ",
            urdf_xacro_file_value,
        ]
        if urdf_xacro_args_value:
            robot_description_cmd += [" ", urdf_xacro_args_value]
        robot_description_command = Command(robot_description_cmd)
        tesseract_urdf_xml = robot_description_command.perform(context)
        tesseract_robot_description = {
            "robot_description": _ensure_tesseract_make_convex_attribute(
                tesseract_urdf_xml, make_convex=True
            )
        }

        robot_description_semantic_cmd = [
            FindExecutable(name="xacro"),
            " ",
            srdf_xacro_file_value,
        ]
        if srdf_xacro_args_value:
            robot_description_semantic_cmd += [" ", srdf_xacro_args_value]
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(
                Command(robot_description_semantic_cmd), value_type=str
            )
        }

        tesseract_monitor_node = Node(
            package="tesseract_monitoring",
            executable="tesseract_monitoring_environment_node",
            namespace=namespace,
            output="screen",
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                tesseract_robot_description,
                robot_description_semantic,
                {
                    "monitor_namespace": monitor_namespace,
                    "monitored_namespace": monitored_namespace,
                    "joint_state_topic": joint_state_topic,
                    "publish_environment": False,
                },
            ],
        )

        scene_bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("rlc_scene_bridge"),
                        "launch",
                        "scene_bridge.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "namespace": namespace,
                "config": bridge_config,
                "monitor_namespace": monitor_namespace,
                "env_name": env_name,
                "scene_topic": scene_topic,
                "get_scene_srv": get_scene_srv,
            }.items(),
        )

        return [tesseract_monitor_node, scene_bridge_launch]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
