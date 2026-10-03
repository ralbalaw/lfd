#include "rlc_executive/bt_nodes/motion/moveit_plan.hpp"

#include <cstddef>
#include <exception>
#include <memory>
#include <string>

#include <behaviortree_ros2/ros_node_params.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/bt_nodes/common/trajectory_conversion.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"

namespace rlc_executive
{
namespace
{

PlanSummary makePlanSummary(PlanStatus status, const std::string& message,
                            int moveit_error_code = 0, double planning_time_sec = 0.0)
{
  PlanSummary out;
  out.status = status;
  out.moveit_error_code = moveit_error_code;
  out.message = message;
  out.planning_time_sec = planning_time_sec;
  return out;
}

void applyPlanningConfig(moveit_msgs::msg::MotionPlanRequest& request,
                         const PlanningConfig& config)
{
  request.pipeline_id = config.pipeline_id;
  request.planner_id = config.planner_id;
  request.allowed_planning_time = config.allowed_planning_time_sec;
  request.num_planning_attempts = config.num_planning_attempts;
  request.max_velocity_scaling_factor = config.max_velocity_scaling_factor;
  request.max_acceleration_scaling_factor = config.max_acceleration_scaling_factor;
}

PlanStatus planStatusFromActionError(BT::ActionNodeErrorCode error)
{
  if (error == BT::SEND_GOAL_TIMEOUT)
  {
    return PlanStatus::kTimeout;
  }
  return PlanStatus::kFailure;
}

std::string makeActionErrorMessage(BT::ActionNodeErrorCode error)
{
  const char* const error_name = BT::toStr(error);
  return std::string("MoveItPlan: action failed: ") +
         (error_name ? error_name : "UNKNOWN");
}

}  // namespace

MoveItPlan::MoveItPlan(const std::string& name, const BT::NodeConfig& config,
                       const BT::RosNodeParams& params,
                       const PlanningConfigRegistry* planning_configs)
  : Base(name, config, params)
  , planning_configs_(bt_utils::requireNonNull(planning_configs, *this,
                                               "PlanningConfigRegistry pointer"))
  , logger_(logger().get_child(bt_utils::sanitizeLoggerName(fullPath())))
{
}

bool MoveItPlan::setGoal(Goal& goal)
{
  try
  {
    last_failure_message_.clear();

    const auto req =
        bt_utils::requireInput<std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest>>(
            *this, PortKeys::kRequest);
    if (!req)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::kRequest, "' is null");
    }

    config_name_ = bt_utils::requireInput<std::string>(*this, PortKeys::kPlanningConfig);

    const PlanningConfig* config = planning_configs_->find(config_name_);
    if (!config)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: unknown planning_config '", config_name_, "'");
    }

    // Clear previous output to avoid stale blackboard values.
    bt_utils::setOutput(*this, PortKeys::kTrajectory,
                        std::shared_ptr<const rbt_types::JointTrajectory>());

    goal.request = *req;
    applyPlanningConfig(goal.request, *config);
    goal.planning_options.plan_only = true;

    RCLCPP_INFO(logger_, "start planning_config='%s' group='%s'", config_name_.c_str(),
                goal.request.group_name.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    last_failure_message_ = e.what();
    return false;
  }
}

BT::NodeStatus MoveItPlan::onResultReceived(const WrappedResult& result)
{
  const auto& moveit_result = result.result;
  if (!moveit_result)
  {
    return failWithError("MoveItPlan: received null MoveGroup result");
  }

  const int moveit_error_code = moveit_result->error_code.val;
  const double planning_time_sec = moveit_result->planning_time;

  if (moveit_error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    bt_utils::setOutput(*this, PortKeys::kPlanResult,
                        makePlanSummary(PlanStatus::kSuccess, "MoveItPlan: plan success",
                                        moveit_error_code, planning_time_sec));

    const std::shared_ptr<const rbt_types::JointTrajectory> trajectory =
        std::make_shared<rbt_types::JointTrajectory>(
            bt_utils::fromRobotTrajectory(moveit_result->planned_trajectory));
    bt_utils::setOutput(*this, PortKeys::kTrajectory, trajectory);

    const std::size_t traj_points = trajectory->length();
    const std::size_t traj_joints = trajectory->dim();
    RCLCPP_INFO(logger_, "SUCCESS (%.3fs) traj_points=%zu traj_joints=%zu",
                planning_time_sec, traj_points, traj_joints);
    return BT::NodeStatus::SUCCESS;
  }

  const std::string message = "MoveItPlan: planning failed with MoveItErrorCodes.val=" +
                              std::to_string(moveit_error_code);
  return failWithError(message, PlanStatus::kFailure, moveit_error_code,
                       planning_time_sec);
}

BT::NodeStatus MoveItPlan::onFailure(BT::ActionNodeErrorCode error)
{
  const std::string message = last_failure_message_.empty() ?
                                  makeActionErrorMessage(error) :
                                  last_failure_message_;
  last_failure_message_.clear();
  return failWithError(message, planStatusFromActionError(error));
}

void MoveItPlan::onHalt()
{
  RCLCPP_WARN(logger_, "halt requested, canceling");
}

BT::NodeStatus MoveItPlan::failWithError(const std::string& msg, PlanStatus status,
                                         int moveit_error_code, double planning_time_sec)
{
  bt_utils::setOutput(*this, PortKeys::kPlanResult,
                      makePlanSummary(status, msg, moveit_error_code, planning_time_sec));
  bt_utils::setOutput(*this, PortKeys::kTrajectory,
                      std::shared_ptr<const rbt_types::JointTrajectory>());
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace rlc_executive
