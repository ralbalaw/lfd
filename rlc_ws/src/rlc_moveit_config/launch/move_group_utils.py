"""Helpers for composing MoveIt `move_group` launch actions."""

from __future__ import annotations

from enum import Enum
import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

DEFAULT_REQUEST_ADAPTERS: list[str] = [
    "default_planning_request_adapters/ResolveConstraintFrames",
    "default_planning_request_adapters/ValidateWorkspaceBounds",
    "default_planning_request_adapters/CheckStartStateBounds",
    "default_planning_request_adapters/CheckStartStateCollision",
]

DEFAULT_RESPONSE_ADAPTERS: list[str] = [
    "default_planning_response_adapters/AddTimeOptimalParameterization",
    "default_planning_response_adapters/ValidateSolution",
    "default_planning_response_adapters/DisplayMotionPath",
]


class Planner(Enum):
    """Enum for supported planners."""

    OMPL = "ompl"
    RLC_TRAJOPT = "rlc_trajopt"


def load_yaml(package_name: str, relative_path: str) -> dict[str, Any]:
    """
    Load a YAML mapping from a package share path.

    Parameters
    ----------
    package_name : str
        ROS package name used to resolve the share directory.
    relative_path : str
        Path relative to the package share directory, e.g.
        ``config/fr3/kinematics.yaml``.

    Returns
    -------
    dict
        Parsed YAML mapping. Empty if the YAML file is empty.

    Raises
    ------
    RuntimeError
        If the file cannot be read or the YAML does not parse to a mapping.
    """
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
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


def planning_plugins_for(planner: Planner) -> list[str]:
    """
    Map a high-level planner selection to MoveIt planner plugin class names.

    Parameters
    ----------
    planner : Planner
        Planner selection.

    Returns
    -------
    list[str]
        Planner plugin class names in the order they are registered.

    Raises
    ------
    ValueError
        If `planner` is unsupported.
    """
    if planner is Planner.OMPL:
        return ["ompl_interface/OMPLPlanner"]
    if planner is Planner.RLC_TRAJOPT:
        return [
            "ompl_interface/OMPLPlanner",
            "rlc_planner/TrajOptPlannerManager",
        ]
    raise ValueError(f"Unsupported planner selection: '{planner}'")


def planning_pipelines_for(planner: Planner) -> list[str]:
    """
    Map a high-level planner selection to MoveIt planning pipeline names.

    Parameters
    ----------
    planner : Planner
        Planner selection.

    Returns
    -------
    list[str]
        Planning pipeline names in the order they are registered.

    Raises
    ------
    ValueError
        If `planner` is unsupported.
    """
    if planner is Planner.OMPL:
        return ["ompl"]
    if planner is Planner.RLC_TRAJOPT:
        return ["ompl", "rlc_trajopt"]
    raise ValueError(f"Unsupported planner selection: '{planner}'")


def make_planning_pipeline_config(
    *,
    planner: Planner,
    planning_configs: dict[Planner, Any],
    request_adapters: list[str] | None = None,
    response_adapters: list[str] | None = None,
    start_state_max_bounds_error: float = 0.1,
) -> dict[str, Any]:
    """
    Build the MoveIt planning pipeline parameter mapping for `move_group`.

    Parameters
    ----------
    planner : Planner
        Planner selection enum.
    planning_configs : dict[Planner, Any]
        Planning parameter mappings keyed by `Planner`.

        The returned mapping follows MoveIt's multi-pipeline layout:
        - `move_group.planning_pipelines` and `move_group.default_planning_pipeline`
        - one top-level parameter namespace per pipeline name (e.g. `ompl`,
          `rlc_trajopt`)
    request_adapters : list[str], optional
        Request adapter plugins. If None, uses package defaults.
    response_adapters : list[str], optional
        Response adapter plugins. If None, uses package defaults.
    start_state_max_bounds_error : float, optional
        Max start-state bounds error.

    Returns
    -------
    dict
        A mapping suitable for passing to `launch_ros.actions.Node(parameters=...)`.
    """
    if request_adapters is None:
        request_adapters = list(DEFAULT_REQUEST_ADAPTERS)
    if response_adapters is None:
        response_adapters = list(DEFAULT_RESPONSE_ADAPTERS)

    pipelines = planning_pipelines_for(planner)
    default_pipeline = pipelines[-1]

    common: dict[str, Any] = {
        "request_adapters": request_adapters,
        "response_adapters": response_adapters,
        "start_state_max_bounds_error": start_state_max_bounds_error,
    }

    params: dict[str, Any] = {
        "planning_pipelines": pipelines,
        "default_planning_pipeline": default_pipeline,
        "ompl": {
            "planning_plugins": planning_plugins_for(Planner.OMPL),
            **common,
            **planning_configs.get(Planner.OMPL, {}),
        },
    }

    if planner is Planner.RLC_TRAJOPT:
        params["rlc_trajopt"] = {
            "planning_plugins": planning_plugins_for(Planner.RLC_TRAJOPT),
            **common,
            # include OMPL params too, because OMPL is stage 1 in this chained pipeline
            **planning_configs.get(Planner.OMPL, {}),
            **planning_configs.get(Planner.RLC_TRAJOPT, {}),
        }

    return params


def make_move_group_node(
    *,
    namespace: LaunchConfiguration,
    use_sim_time: bool | LaunchConfiguration = False,
    robot_description: dict[str, Any],
    robot_description_semantic: dict[str, Any],
    kinematics: dict[str, Any],
    joint_limits: dict[str, Any],
    controllers: dict[str, Any],
    planning_configs: dict[Planner, Any],
    planner: Planner,
) -> Node:
    """
    Create the `move_group` node action.

    Parameters
    ----------
    namespace : LaunchConfiguration
        ROS namespace for all nodes.
    use_sim_time : bool or LaunchConfiguration, optional
        Whether to use simulation time for `move_group`.
    robot_description : dict
        `robot_description` parameter mapping.
    robot_description_semantic : dict
        `robot_description_semantic` parameter mapping.
    kinematics : dict
        MoveIt kinematics YAML contents.
    joint_limits : dict
        MoveIt joint limits YAML contents.
    controllers : dict
        MoveIt simple controller manager YAML contents.
    planning_configs : dict[Planner, Any]
        Planning YAML contents.
    planner : Planner
        Planner selection enum.

    Returns
    -------
    Node
        A configured `move_group` node action.
    """
    kinematics_config: dict[str, Any] = {"robot_description_kinematics": kinematics}
    joint_limits_config: dict[str, Any] = {"robot_description_planning": joint_limits}

    planning_pipeline_config = make_planning_pipeline_config(
        planner=planner,
        planning_configs=planning_configs,
    )

    moveit_controllers: dict[str, Any] = {
        "moveit_simple_controller_manager": controllers,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    trajectory_execution: dict[str, Any] = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters: dict[str, Any] = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
