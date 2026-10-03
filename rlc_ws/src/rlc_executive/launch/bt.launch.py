import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_yaml(package_name: str, relative_path: str) -> dict:
    """
    Load a YAML mapping from a package share path.

    Parameters
    ----------
    package_name : str
        ROS package name used to resolve the share directory.
    relative_path : str
        YAML path relative to the package share directory, or an absolute path.

    Returns
    -------
    dict
        Parsed YAML mapping. Empty if the YAML file is empty.

    Raises
    ------
    RuntimeError
        If the file cannot be read or the YAML does not parse to a mapping.
    """
    if os.path.isabs(relative_path):
        absolute_path = relative_path
    else:
        absolute_path = os.path.join(
            get_package_share_directory(package_name), relative_path
        )

    try:
        with open(absolute_path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
    except EnvironmentError as ex:
        raise RuntimeError(f"Could not read YAML file: {absolute_path}") from ex
    except yaml.YAMLError as ex:
        raise RuntimeError(f"Could not parse YAML file: {absolute_path}") from ex

    if not isinstance(data, dict):
        raise RuntimeError(
            f"Expected YAML mapping in '{absolute_path}', got '{type(data).__name__}'"
        )

    return data


def _resolve_paths(context, *args, **kwargs):
    """
    Resolve executive parameter, planning config, and BehaviorTree XML paths.

    Rules (applied per argument):
    - If the argument is an absolute path, use it as-is.
    - If the argument contains a directory component, treat it as relative to the
      package share directory.
    - Otherwise, treat it as a bare filename under a package share subdirectory
      (e.g. `config/`, `bt_trees/`).

    Parameters
    ----------
    context : launch.LaunchContext
        Launch context used to evaluate substitutions.

    Returns
    -------
    list[launch.Action]
        Actions that set resolved launch configurations.
    """
    pkg_share_path = FindPackageShare("rlc_executive").perform(context)

    def resolve_into_subdir(arg_name: str, subdir: str) -> str:
        arg_value = LaunchConfiguration(arg_name).perform(context)
        if os.path.isabs(arg_value):
            return arg_value
        if os.path.dirname(arg_value):
            return os.path.join(pkg_share_path, arg_value)
        return os.path.join(pkg_share_path, subdir, arg_value)

    return [
        SetLaunchConfiguration(
            "bt_executive_config_resolved",
            resolve_into_subdir("bt_executive_config", "config"),
        ),
        SetLaunchConfiguration(
            "planning_configs_path_resolved",
            resolve_into_subdir("planning_configs_path", "config"),
        ),
        SetLaunchConfiguration(
            "default_tree_resolved", resolve_into_subdir("default_tree", "bt_trees")
        ),
    ]


def generate_launch_description():
    """
    Launch the BehaviorTree executive with configurable config and tree paths.

    Returns
    -------
    launch.LaunchDescription
        Launch description for the BT executive node.
    """
    bt_executive_config = LaunchConfiguration("bt_executive_config_resolved")
    planning_configs_path = LaunchConfiguration("planning_configs_path_resolved")
    default_tree_resolved = LaunchConfiguration("default_tree_resolved")
    log_level = LaunchConfiguration("log_level")
    enable_groot2 = LaunchConfiguration("enable_groot2")
    groot2_port = LaunchConfiguration("groot2_port")
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_xacro_file = LaunchConfiguration("urdf_xacro_file")
    urdf_xacro_args = LaunchConfiguration("urdf_xacro_args")
    srdf_xacro_file = LaunchConfiguration("srdf_xacro_file")
    srdf_xacro_args = LaunchConfiguration("srdf_xacro_args")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    kinematics_yaml = LaunchConfiguration("kinematics_yaml")
    joint_limits_yaml = LaunchConfiguration("joint_limits_yaml")

    def launch_setup(context, *args, **kwargs):  # noqa: ANN001, ARG001
        """
        Create the BT executive action after resolving robot model inputs.

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
            Launch actions for the BT executive node.

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
        robot_description = {
            "robot_description": ParameterValue(
                Command(robot_description_cmd), value_type=str
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

        moveit_config_package_value = moveit_config_package.perform(context)
        kinematics = _load_yaml(
            moveit_config_package_value, kinematics_yaml.perform(context)
        )
        joint_limits = _load_yaml(
            moveit_config_package_value, joint_limits_yaml.perform(context)
        )

        return [
            Node(
                package="rlc_executive",
                executable="rlc_bt_executive",
                name="bt_executive",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    bt_executive_config,
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "planning_configs_path": planning_configs_path,
                        "default_tree": default_tree_resolved,
                        "enable_groot2": ParameterValue(
                            enable_groot2, value_type=bool
                        ),
                        "groot2_port": ParameterValue(groot2_port, value_type=int),
                    },
                    robot_description,
                    robot_description_semantic,
                    {"robot_description_kinematics": kinematics},
                    {"robot_description_planning": joint_limits},
                ],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bt_executive_config",
                default_value="bt_executive.yaml",
                description="ROS parameter YAML file for the BT executive node.",
            ),
            DeclareLaunchArgument(
                "planning_configs_path",
                default_value="planning_configs.yaml",
                description="Planning configs YAML file loaded directly by the executive.",
            ),
            DeclareLaunchArgument("default_tree", default_value="reactive_core.xml"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("enable_groot2", default_value="false"),
            DeclareLaunchArgument("groot2_port", default_value="1667"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true.",
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
                "moveit_config_package",
                default_value="rlc_moveit_config",
                description="Package that provides MoveIt robot config YAML files.",
            ),
            DeclareLaunchArgument(
                "kinematics_yaml",
                default_value="config/fr3/kinematics.yaml",
                description="Path to kinematics YAML relative to moveit_config_package.",
            ),
            DeclareLaunchArgument(
                "joint_limits_yaml",
                default_value="config/fr3/joint_limits.yaml",
                description="Path to joint limits YAML relative to moveit_config_package.",
            ),
            OpaqueFunction(function=_resolve_paths),
            OpaqueFunction(function=launch_setup),
        ]
    )
