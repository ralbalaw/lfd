#include "rlc_executive/bt_nodes/motion/time_parameterize_moveit.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/bt_nodes/common/trajectory_conversion.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"

namespace rlc_executive
{

namespace
{
robot_trajectory::RobotTrajectory
buildRobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                     const moveit::core::RobotState& reference_state,
                     const rbt_types::JointPath& path, const std::string& group_name)
{
  robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);
  const auto* group = trajectory.getGroup();
  if (!group)
  {
    throw std::runtime_error("unknown MoveIt joint model group: '" + group_name + "'");
  }

  if (path.length() < 2U)
  {
    throw std::runtime_error("path must contain at least two joint waypoints");
  }

  const std::vector<std::string>& joint_names = group->getVariableNames();
  if (path.dim() != joint_names.size())
  {
    throw std::runtime_error("path joint dimension does not match MoveIt group '" +
                             group_name + "'");
  }

  const moveit_msgs::msg::RobotTrajectory path_msg =
      bt_utils::toRobotTrajectory(path, joint_names);

  trajectory.setRobotTrajectoryMsg(reference_state, path_msg);
  return trajectory;
}

std::shared_ptr<const rbt_types::JointTrajectory>
parameterize(robot_trajectory::RobotTrajectory& trajectory,
             double max_velocity_scaling_factor, double max_acceleration_scaling_factor)
{
  if (!trajectory.getGroup())
  {
    throw std::runtime_error("MoveIt time parameterization requires a valid joint "
                             "model group");
  }
  if (trajectory.getWayPointCount() < 2U)
  {
    throw std::runtime_error("MoveIt time parameterization requires at least two "
                             "waypoints");
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(trajectory, max_velocity_scaling_factor,
                              max_acceleration_scaling_factor))
  {
    throw std::runtime_error(
        "MoveIt TimeOptimalTrajectoryGeneration failed to time-parameterize the path");
  }

  moveit_msgs::msg::RobotTrajectory output_msg;
  trajectory.getRobotTrajectoryMsg(output_msg);
  return std::make_shared<rbt_types::JointTrajectory>(
      bt_utils::fromRobotTrajectory(output_msg));
}
}  // namespace

TimeParameterizeMoveIt::TimeParameterizeMoveIt(
    const std::string& name, const BT::NodeConfig& config, rclcpp::Logger logger,
    moveit::core::RobotModelConstPtr robot_model,
    const moveit::core::RobotState* reference_state,
    const PlanningConfigRegistry* planning_configs)
  : BT::SyncActionNode(name, config)
  , logger_(logger.get_child(bt_utils::sanitizeLoggerName(fullPath())))
  , robot_model_(bt_utils::requireNonNull(std::move(robot_model), *this, "robot_model"))
  , reference_state_(bt_utils::requireNonNull(reference_state, *this, "reference_state"))
  , planning_configs_(
        bt_utils::requireNonNull(planning_configs, *this, "planning_configs"))
{
}

BT::NodeStatus TimeParameterizeMoveIt::tick()
{
  try
  {
    const auto path =
        bt_utils::requireInput<std::shared_ptr<const JointPath>>(*this, PortKeys::kPath);

    if (!path)
    {
      throw std::runtime_error("path input is nullptr");
    }

    const auto group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::kGroupName);

    const auto planning_config =
        bt_utils::requireInput<std::string>(*this, PortKeys::kPlanningConfig);

    const auto* config = planning_configs_->find(planning_config);

    if (!config)
    {
      throw std::runtime_error("planning config '" + planning_config + "' not found");
    }

    auto robot_trajectory =
        buildRobotTrajectory(robot_model_, *reference_state_, *path, group_name);
    const auto trajectory =
        parameterize(robot_trajectory, config->max_velocity_scaling_factor,
                     config->max_acceleration_scaling_factor);

    bt_utils::setOutput(*this, PortKeys::kTrajectory, trajectory);
    bt_utils::setOutput(*this, PortKeys::kError, std::string());

    RCLCPP_INFO(logger_, "time-parameterized path with %zu points to %zu points",
                path->length(), trajectory->length());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus TimeParameterizeMoveIt::failWithError(const std::string& msg)
{
  bt_utils::setOutput(*this, PortKeys::kTrajectory,
                      std::shared_ptr<const JointTrajectory>());
  bt_utils::setOutput(*this, PortKeys::kError, msg);
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}
}  // namespace rlc_executive
