from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch FR3 in Gazebo Sim, MoveIt `move_group`, and optional Servo teleoperation.

    Notes
    -----
    This is a composition launch that includes:
    - `rlc_bringup/launch/fr3_gz_control.launch.py` (Gazebo + ros2_control + controllers)
    - `rlc_moveit_config/launch/move_group.launch.py` (MoveIt)
    - `rlc_moveit_config/launch/servo.launch.py` (Servo + game controller) when
      `launch_servo:=true`
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    xacro_file = LaunchConfiguration("xacro_file")
    srdf_xacro_file = LaunchConfiguration("srdf_xacro_file")
    tesseract_srdf_xacro_file = LaunchConfiguration("tesseract_srdf_xacro_file")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")
    controller_manager = LaunchConfiguration("controller_manager")
    arm_controller = LaunchConfiguration("arm_controller")
    gripper_controller = LaunchConfiguration("gripper_controller")

    moveit_namespace = LaunchConfiguration("namespace")
    planner = LaunchConfiguration("planner")
    launch_servo = LaunchConfiguration("launch_servo")
    launch_executive = LaunchConfiguration("launch_executive")
    launch_demo = LaunchConfiguration("launch_demo")

    move_group_urdf_xacro_args = LaunchConfiguration("move_group_urdf_xacro_args")
    move_group_srdf_xacro_args = LaunchConfiguration("move_group_srdf_xacro_args")
    tesseract_srdf_xacro_args = LaunchConfiguration("tesseract_srdf_xacro_args")

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_kinematics_yaml = LaunchConfiguration("kinematics_yaml")
    moveit_joint_limits_yaml = LaunchConfiguration("joint_limits_yaml")
    moveit_controllers_yaml = LaunchConfiguration("moveit_controllers_yaml")
    moveit_ompl_planning_yaml = LaunchConfiguration("ompl_planning_yaml")
    moveit_trajopt_planning_yaml = LaunchConfiguration("trajopt_planning_yaml")
    tesseract_monitor_namespace = LaunchConfiguration("tesseract_monitor_namespace")
    tesseract_env_name = LaunchConfiguration("tesseract_env_name")
    tesseract_joint_state_topic = LaunchConfiguration("tesseract_joint_state_topic")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    bt_executive_config = LaunchConfiguration("bt_executive_config")
    planning_configs_path = LaunchConfiguration("planning_configs_path")
    default_tree = LaunchConfiguration("default_tree")
    executive_log_level = LaunchConfiguration("executive_log_level")
    enable_groot2 = LaunchConfiguration("enable_groot2")
    groot2_port = LaunchConfiguration("groot2_port")

    demo_config = LaunchConfiguration("demo_config")
    demo_log_level = LaunchConfiguration("demo_log_level")

    gz_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_bringup"),
                    "launch",
                    "fr3_gz_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "robot_name": robot_name,
            "xacro_file": xacro_file,
            "controllers_yaml": controllers_yaml,
            "load_gripper": load_gripper,
            "ee_id": ee_id,
            "controller_manager": controller_manager,
            "arm_controller": arm_controller,
            "gripper_controller": gripper_controller,
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": moveit_namespace,
            "planner": planner,
            "moveit_config_package": moveit_config_package,
            "kinematics_yaml": moveit_kinematics_yaml,
            "joint_limits_yaml": moveit_joint_limits_yaml,
            "moveit_controllers_yaml": moveit_controllers_yaml,
            "ompl_planning_yaml": moveit_ompl_planning_yaml,
            "trajopt_planning_yaml": moveit_trajopt_planning_yaml,
            "tesseract_monitor_namespace": tesseract_monitor_namespace,
            "tesseract_env_name": tesseract_env_name,
            "urdf_xacro_file": xacro_file,
            "urdf_xacro_args": move_group_urdf_xacro_args,
            "srdf_xacro_file": srdf_xacro_file,
            "srdf_xacro_args": move_group_srdf_xacro_args,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    tesseract_scene_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_scene_bridge"),
                    "launch",
                    "tesseract_scene_sync.launch.py",
                ]
            )
        ),
        condition=IfCondition(
            PythonExpression(["'", planner, "' == 'rlc_trajopt'"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": moveit_namespace,
            "urdf_xacro_file": xacro_file,
            "urdf_xacro_args": move_group_urdf_xacro_args,
            "srdf_xacro_file": tesseract_srdf_xacro_file,
            "srdf_xacro_args": tesseract_srdf_xacro_args,
            "monitor_namespace": tesseract_monitor_namespace,
            "env_name": tesseract_env_name,
            "joint_state_topic": tesseract_joint_state_topic,
        }.items(),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_moveit_config"),
                    "launch",
                    "servo.launch.py",
                ]
            )
        ),
        condition=IfCondition(launch_servo),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": moveit_namespace,
            "moveit_config_package": moveit_config_package,
            "kinematics_yaml": moveit_kinematics_yaml,
            "joint_limits_yaml": moveit_joint_limits_yaml,
            "urdf_xacro_file": xacro_file,
            "urdf_xacro_args": move_group_urdf_xacro_args,
            "srdf_xacro_file": srdf_xacro_file,
            "srdf_xacro_args": move_group_srdf_xacro_args,
        }.items(),
    )

    executive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_executive"),
                    "launch",
                    "bt.launch.py",
                ]
            )
        ),
        condition=IfCondition(launch_executive),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "bt_executive_config": bt_executive_config,
            "planning_configs_path": planning_configs_path,
            "default_tree": default_tree,
            "log_level": executive_log_level,
            "enable_groot2": enable_groot2,
            "groot2_port": groot2_port,
            "moveit_config_package": moveit_config_package,
            "kinematics_yaml": moveit_kinematics_yaml,
            "joint_limits_yaml": moveit_joint_limits_yaml,
            "urdf_xacro_file": xacro_file,
            "urdf_xacro_args": move_group_urdf_xacro_args,
            "srdf_xacro_file": srdf_xacro_file,
            "srdf_xacro_args": move_group_srdf_xacro_args,
        }.items(),
    )

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_frontend"),
                    "launch",
                    "collect_demonstration.launch.py",
                ]
            )
        ),
        condition=IfCondition(launch_demo),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config": demo_config,
            "log_level": demo_log_level,
        }.items(),
    )

    return LaunchDescription(
        [
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
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("franka_description"),
                        "robots",
                        "fr3",
                        "fr3.urdf.xacro",
                    ]
                ),
                description="Robot URDF Xacro file.",
            ),
            DeclareLaunchArgument(
                "srdf_xacro_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("franka_description"),
                        "robots",
                        "fr3",
                        "fr3.srdf.xacro",
                    ]
                ),
                description="Robot SRDF Xacro file.",
            ),
            DeclareLaunchArgument(
                "tesseract_srdf_xacro_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rlc_moveit_config"),
                        "config",
                        "fr3",
                        "fr3_tesseract.srdf.xacro",
                    ]
                ),
                description="Tesseract-compatible robot SRDF Xacro file.",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rlc_bringup"),
                        "config",
                        "fr3_gz_controllers.yaml",
                    ]
                ),
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
                "move_group_urdf_xacro_args",
                default_value=PythonExpression(
                    [
                        '"ros2_control:=false hand:=" + "',
                        load_gripper,
                        '" + " ee_id:=" + "',
                        ee_id,
                        '" + " robot_type:=fr3"',
                    ]
                ),
                description="URDF xacro args used by MoveIt (FR3-specific).",
            ),
            DeclareLaunchArgument(
                "move_group_srdf_xacro_args",
                default_value=PythonExpression(
                    [
                        '"hand:=" + "',
                        load_gripper,
                        '" + " ee_id:=" + "',
                        ee_id,
                        '"',
                    ]
                ),
                description="SRDF xacro args used by MoveIt (FR3-specific).",
            ),
            DeclareLaunchArgument(
                "tesseract_srdf_xacro_args",
                default_value=PythonExpression(
                    [
                        '"hand:=" + "',
                        load_gripper,
                        '" + " ee_id:=" + "',
                        ee_id,
                        '"',
                    ]
                ),
                description="SRDF xacro args used by Tesseract (FR3-specific).",
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
                description="Controller name to spawn for the gripper.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for MoveIt nodes.",
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="ompl",
                description="Planner pipeline selection (ompl | rlc_trajopt).",
            ),
            DeclareLaunchArgument(
                "launch_servo",
                default_value="true",
                description="Launch MoveIt Servo and the game controller node if true.",
            ),
            DeclareLaunchArgument(
                "launch_executive",
                default_value="false",
                description="Launch the BehaviorTree executive if true.",
            ),
            DeclareLaunchArgument(
                "launch_demo",
                default_value="false",
                description="Launch the demonstration collection server if true.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz if true.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="rviz/moveit.rviz",
                description=(
                    "RViz config file path. If relative, it is resolved relative to "
                    "`moveit_config_package`."
                ),
            ),
            DeclareLaunchArgument(
                "moveit_config_package",
                default_value="rlc_moveit_config",
                description="Package that provides the MoveIt config YAML files.",
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
            DeclareLaunchArgument(
                "moveit_controllers_yaml",
                default_value="config/fr3/controllers_gz.yaml",
                description="MoveIt controller manager YAML (Gazebo).",
            ),
            DeclareLaunchArgument(
                "ompl_planning_yaml",
                default_value="config/fr3/ompl_planning.yaml",
                description="Path to OMPL planning YAML relative to moveit_config_package.",
            ),
            DeclareLaunchArgument(
                "trajopt_planning_yaml",
                default_value="config/fr3/trajopt_planning.yaml",
                description="Path to TrajOpt parameters YAML relative to moveit_config_package.",
            ),
            DeclareLaunchArgument(
                "tesseract_monitor_namespace",
                default_value="tesseract_monitor",
                description=(
                    "Tesseract monitor namespace used for services/topics (e.g., "
                    '"/<ns>/get_tesseract_information").'
                ),
            ),
            DeclareLaunchArgument(
                "tesseract_env_name",
                default_value="fr3",
                description="Tesseract environment name used by monitor clients.",
            ),
            DeclareLaunchArgument(
                "tesseract_joint_state_topic",
                default_value="",
                description=(
                    "Joint states topic name for the Tesseract monitor. If empty, "
                    "the monitor uses its internal default (typically '/joint_states')."
                ),
            ),
            DeclareLaunchArgument(
                "bt_executive_config",
                default_value="bt_executive.yaml",
                description="BT executive ROS parameter YAML path or filename.",
            ),
            DeclareLaunchArgument(
                "planning_configs_path",
                default_value="planning_configs.yaml",
                description="BT planning configs YAML path or filename loaded by C++.",
            ),
            DeclareLaunchArgument(
                "default_tree",
                default_value="reactive_core.xml",
                description="Default BehaviorTree XML path or filename.",
            ),
            DeclareLaunchArgument(
                "executive_log_level",
                default_value="info",
                description="Log level for the BT executive.",
            ),
            DeclareLaunchArgument(
                "enable_groot2",
                default_value="false",
                description="Enable Groot2 publisher for the BT executive.",
            ),
            DeclareLaunchArgument(
                "groot2_port",
                default_value="1667",
                description="Groot2 publisher TCP port.",
            ),
            DeclareLaunchArgument(
                "demo_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rlc_frontend"),
                        "config",
                        "collect_demonstration.yaml",
                    ]
                ),
                description="Demonstration collection server config YAML.",
            ),
            DeclareLaunchArgument(
                "demo_log_level",
                default_value="info",
                description="Log level for the demonstration collection server.",
            ),
            gz_control_launch,
            GroupAction(actions=[tesseract_scene_sync_launch], scoped=True),
            move_group_launch,
            servo_launch,
            executive_launch,
            demo_launch,
        ]
    )
