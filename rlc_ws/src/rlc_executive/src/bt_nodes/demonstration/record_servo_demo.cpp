#include "rlc_executive/bt_nodes/demonstration/record_servo_demo.hpp"

#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/bt_nodes/common/trajectory_conversion.hpp"

namespace rlc_executive
{
namespace
{

std::string makeActionErrorMessage(BT::ActionNodeErrorCode error)
{
  const char* const error_name = BT::toStr(error);
  return std::string("RecordServoDemo: action failed: ") +
         (error_name ? error_name : "UNKNOWN");
}

}  // namespace

RecordServoDemo::RecordServoDemo(const std::string& name, const BT::NodeConfig& config,
                                 const BT::RosNodeParams& params)
  : Base(name, config, params)
  , logger_(logger().get_child(bt_utils::sanitizeLoggerName(fullPath())))
{
}

bool RecordServoDemo::setGoal(Goal& goal)
{
  try
  {
    last_failure_message_.clear();
    start_time_ = now();

    goal.sample_dt = bt_utils::requireInput<double>(*this, PortKeys::kSampleDt);
    if (goal.sample_dt <= 0.0)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: sample_dt must be > 0");
    }

    bt_utils::setOutput(*this, PortKeys::kDemonstration,
                        std::shared_ptr<const JointTrajectory>());
    bt_utils::setRunningDiagnostics(*this, "collecting demonstration", 0.0);

    RCLCPP_INFO(logger_, "start sample_dt=%.6f", goal.sample_dt);
    return true;
  }
  catch (const std::exception& e)
  {
    last_failure_message_ = e.what();
    return false;
  }
}

BT::NodeStatus RecordServoDemo::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  if (!feedback)
  {
    return BT::NodeStatus::RUNNING;
  }

  const std::string feedback_msg =
      feedback->feedback.empty() ? "collecting demonstration" : feedback->feedback;
  bt_utils::setRunningDiagnostics(*this, feedback_msg, feedback->elapsed_sec);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecordServoDemo::onResultReceived(const WrappedResult& result)
{
  const auto action_result = result.result;
  if (!action_result)
  {
    return failWithError("RecordServoDemo: received null CollectDemonstration result",
                         elapsedSec());
  }

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    return failWithError(action_result->error.empty() ?
                             "RecordServoDemo: CollectDemonstration failed" :
                             action_result->error,
                         action_result->elapsed_sec);
  }

  try
  {
    auto demonstration = std::make_shared<JointTrajectory>(
        bt_utils::fromRobotTrajectory(action_result->demonstration));

    bt_utils::setOutput(*this, PortKeys::kDemonstration,
                        std::shared_ptr<const JointTrajectory>(demonstration));
    bt_utils::setSuccessDiagnostics(*this, action_result->elapsed_sec);

    RCLCPP_INFO(logger_, "SUCCESS (%.3fs, samples=%u)", action_result->elapsed_sec,
                action_result->sample_count);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what(), action_result->elapsed_sec);
  }
}

BT::NodeStatus RecordServoDemo::onFailure(BT::ActionNodeErrorCode error)
{
  const std::string message = last_failure_message_.empty() ?
                                  makeActionErrorMessage(error) :
                                  last_failure_message_;
  last_failure_message_.clear();
  return failWithError(message, elapsedSec());
}

void RecordServoDemo::onHalt()
{
  RCLCPP_WARN(logger_, "halt requested, canceling demonstration collection");
}

double RecordServoDemo::elapsedSec()
{
  return (now() - start_time_).seconds();
}

BT::NodeStatus RecordServoDemo::failWithError(const std::string& msg, double elapsed_sec)
{
  bt_utils::setOutput(*this, PortKeys::kDemonstration,
                      std::shared_ptr<const JointTrajectory>());
  bt_utils::setFailureDiagnostics(*this, msg, elapsed_sec);
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace rlc_executive
