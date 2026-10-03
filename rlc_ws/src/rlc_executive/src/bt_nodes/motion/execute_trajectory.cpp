#include "rlc_executive/bt_nodes/motion/execute_trajectory.hpp"

#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <string_view>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/bt_nodes/common/robot_model_utils.hpp"
#include "rlc_executive/bt_nodes/common/trajectory_conversion.hpp"

namespace rlc_executive
{
namespace
{
ExecResult makeExecResult(ExecStatus status, const std::string& message,
                          int moveit_error_code = 0, double execution_time_sec = 0.0)
{
  ExecResult er;
  er.status = status;
  er.message = message;
  er.error_code = moveit_error_code;
  er.execution_time_sec = execution_time_sec;
  return er;
}

ExecStatus execStatusFromActionError(BT::ActionNodeErrorCode error)
{
  if (error == BT::SEND_GOAL_TIMEOUT)
  {
    return ExecStatus::kTimeout;
  }

  const char* const error_name = BT::toStr(error);
  if (error_name && std::string_view(error_name).find("CANCEL") != std::string_view::npos)
  {
    return ExecStatus::kCanceled;
  }

  return ExecStatus::kFailure;
}

std::string makeActionErrorMessage(BT::ActionNodeErrorCode error)
{
  const char* const error_name = BT::toStr(error);
  return std::string("ExecuteTrajectory: action failed: ") +
         (error_name ? error_name : "UNKNOWN");
}

}  // namespace

ExecuteTrajectory::ExecuteTrajectory(const std::string& name,
                                     const BT::NodeConfig& config,
                                     const BT::RosNodeParams& params,
                                     moveit::core::RobotModelConstPtr robot_model)
  : Base(name, config, params)
  , logger_(logger().get_child(bt_utils::sanitizeLoggerName(fullPath())))
  , robot_model_(std::move(robot_model))
{
}

bool ExecuteTrajectory::setGoal(Goal& goal)
{
  try
  {
    last_failure_message_.clear();

    const auto joint_traj =
        bt_utils::requireInput<std::shared_ptr<const rbt_types::JointTrajectory>>(
            *this, PortKeys::kTrajectory);
    if (!joint_traj)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::kTrajectory, "' is null");
    }

    const auto group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::kGroupName);

    const std::vector<std::string> joint_names =
        bt_utils::getActiveJointNames(*robot_model_, group_name);

    const moveit_msgs::msg::RobotTrajectory robot_traj_msg =
        bt_utils::toRobotTrajectory(*joint_traj, joint_names);

    config_name_ = bt_utils::requireInput<std::string>(*this, PortKeys::kPlanningConfig);

    const std::size_t traj_points = robot_traj_msg.joint_trajectory.points.size();
    const std::size_t traj_joints = robot_traj_msg.joint_trajectory.joint_names.size();

    goal.trajectory = robot_traj_msg;
    start_time_ = now();

    bt_utils::setRunningDiagnostics(*this, "goal_sent", 0.0);

    RCLCPP_INFO(logger_, "start planning_config='%s' traj_points=%zu traj_joints=%zu",
                config_name_.c_str(), traj_points, traj_joints);
    return true;
  }
  catch (const std::exception& e)
  {
    last_failure_message_ = e.what();
    return false;
  }
}

BT::NodeStatus ExecuteTrajectory::onResultReceived(const WrappedResult& result)
{
  const auto& res = result.result;
  if (!res)
  {
    return failWithError("ExecuteTrajectory: received null ExecuteTrajectory result",
                         ExecStatus::kFailure, 0, elapsedSec());
  }

  const int err = res->error_code.val;
  const double exec_time_s = elapsedSec();

  if (err == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    bt_utils::setOutput(*this, PortKeys::kExecResult,
                        makeExecResult(ExecStatus::kSuccess,
                                       "ExecuteTrajectory: execution success", err,
                                       exec_time_s));
    bt_utils::setSuccessDiagnostics(*this, exec_time_s);
    RCLCPP_INFO(logger_, "SUCCESS (%.3fs)", exec_time_s);
    return BT::NodeStatus::SUCCESS;
  }

  const std::string message =
      "ExecuteTrajectory: execution returned MoveItErrorCodes.val=" + std::to_string(err);
  return failWithError(message, ExecStatus::kFailure, err, exec_time_s);
}

BT::NodeStatus ExecuteTrajectory::onFailure(BT::ActionNodeErrorCode error)
{
  const std::string message = last_failure_message_.empty() ?
                                  makeActionErrorMessage(error) :
                                  last_failure_message_;
  last_failure_message_.clear();
  return failWithError(message, execStatusFromActionError(error), 0, elapsedSec());
}

void ExecuteTrajectory::onHalt()
{ RCLCPP_WARN(logger_, "halt requested, canceling"); }

double ExecuteTrajectory::elapsedSec()
{ return (now() - start_time_).seconds(); }

BT::NodeStatus ExecuteTrajectory::failWithError(const std::string& msg, ExecStatus status,
                                                int moveit_error_code,
                                                double execution_time_sec)
{
  bt_utils::setOutput(*this, PortKeys::kExecResult,
                      makeExecResult(status, msg, moveit_error_code, execution_time_sec));
  bt_utils::setFailureDiagnostics(*this, msg, execution_time_sec);

  if (status == ExecStatus::kCanceled)
  {
    RCLCPP_WARN(logger_, "CANCELED: %s", msg.c_str());
  }
  else
  {
    RCLCPP_ERROR(logger_, "FAILURE (code=%d): %s", moveit_error_code, msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace rlc_executive
