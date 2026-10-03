from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch FR3 in Gazebo Sim with ros2_control (effort) and a MoveIt-ready arm controller.

    Notes
    -----
    This is the "Part 1" bringup: simulation + controllers only.

    - Spawns FR3 using `franka_description` with `gazebo_effort:=true` so effort command
      interfaces are available.
    - Publishes a static TF frame `base` (identity w.r.t. `fr3_link0`) to match the
      MoveIt SRDF virtual joint (MoveIt expects the planning frame `base`).
    - Starts `joint_state_broadcaster` and `arm_controller`
      (`joint_trajectory_controller/JointTrajectoryController`) so MoveIt can later
      execute trajectories via `FollowJointTrajectory`.
    - Optionally starts `gripper_controller`
      (`parallel_gripper_action_controller/GripperActionController`) when `load_gripper:=true`.

    Returns
    -------
    LaunchDescription
        Launch description for Gazebo + robot spawn + ros2_control controllers.
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    xacro_file = LaunchConfiguration("xacro_file")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")

    controller_manager = LaunchConfiguration("controller_manager")
    arm_controller = LaunchConfiguration("arm_controller")
    gripper_controller = LaunchConfiguration("gripper_controller")

    gz_system_plugins = PathJoinSubstitution(
        [FindPackagePrefix("gz_ros2_control"), "lib"]
    )
    franka_gz_system_plugins = PathJoinSubstitution(
        [FindPackagePrefix("franka_gz_ros2_control"), "lib"]
    )
    gz_franka_resources = PathJoinSubstitution(
        [FindPackageShare("franka_description"), ".."]
    )

    default_xacro = PathJoinSubstitution(
        [
            FindPackageShare("franka_description"),
            "robots",
            "fr3",
            "fr3.urdf.xacro",
        ]
    )
    default_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("rlc_bringup"), "config", "fr3_gz_controllers.yaml"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " ",
                    "gazebo:=true",
                    " ",
                    "ros2_control:=true",
                    " ",
                    "gazebo_effort:=true",
                    " ",
                    "hand:=",
                    load_gripper,
                    " ",
                    "ee_id:=",
                    ee_id,
                ]
            ),
            value_type=str,
        )
    }

    gz = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", "-r", world],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    base_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fr3_base_frame_tf",
        output="screen",
        arguments=["--frame-id", "fr3_link0", "--child-frame-id", "base"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", robot_name, "-param", "robot_description"],
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager,
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "60",
        ],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            arm_controller,
            "--controller-manager",
            controller_manager,
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "60",
        ],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            gripper_controller,
            "--controller-manager",
            controller_manager,
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "60",
        ],
        condition=IfCondition(load_gripper),
    )

    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
    )
    start_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )
    start_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner, on_exit=[gripper_spawner])
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=gz_system_plugins,
                prepend=True,
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=franka_gz_system_plugins,
                prepend=True,
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=gz_franka_resources,
                prepend=True,
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty.sdf",
                description="Gazebo world SDF (path or resource name).",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="fr3",
                description="Entity name to spawn in Gazebo.",
            ),
            DeclareLaunchArgument(
                "xacro_file",
                default_value=default_xacro,
                description="Robot URDF Xacro file.",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=default_controllers_yaml,
                description="ros2_control controller manager YAML.",
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="true",
                description="Whether to include the gripper in the Gazebo model.",
            ),
            DeclareLaunchArgument(
                "ee_id",
                default_value="franka_hand",
                description="End-effector id for FR3 xacros (e.g., franka_hand, none).",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Controller manager node name or namespace.",
            ),
            DeclareLaunchArgument(
                "arm_controller",
                default_value="arm_controller",
                description="Controller name to spawn for the arm.",
            ),
            DeclareLaunchArgument(
                "gripper_controller",
                default_value="gripper_controller",
                description="Controller name to spawn for the gripper (requires load_gripper:=true).",
            ),
            SetParameter(name="use_sim_time", value=use_sim_time),
            gz,
            clock_bridge,
            robot_state_publisher,
            base_frame_tf,
            spawn,
            start_jsb_after_spawn,
            start_arm_after_jsb,
            start_gripper_after_arm,
        ]
    )
