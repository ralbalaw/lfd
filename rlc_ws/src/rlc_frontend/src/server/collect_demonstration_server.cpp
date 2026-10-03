#include "rlc_frontend/server/collect_demonstration_server.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include <moveit_msgs/msg/servo_status.hpp>

#include "recording/joint_demonstration_recorder.hpp"
#include "rlc_frontend/frontend/frontend_config_parameters.hpp"
#include "rlc_frontend/frontend/types.hpp"

namespace rlc_frontend
{
namespace
{

bool isHardServoHalt(std::int8_t code)
{
  return code == moveit_msgs::msg::ServoStatus::INVALID ||
         code == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY ||
         code == moveit_msgs::msg::ServoStatus::HALT_FOR_COLLISION ||
         code == moveit_msgs::msg::ServoStatus::JOINT_BOUND;
}

std::string makeServoHaltMessage(const moveit_msgs::msg::ServoStatus& status)
{
  if (!status.message.empty())
  {
    return "servo halted: " + status.message;
  }
  return "servo halted with status code " + std::to_string(status.code);
}

double elapsedSec(std::chrono::steady_clock::time_point started_at)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
      .count();
}

CollectDemonstrationConfig loadServerConfig(const rlc_frontend::Params& params)
{
  CollectDemonstrationConfig loaded_config =
      config::loadCollectDemonstrationConfigFromParameters(params);
  if (loaded_config.action_name.empty())
  {
    throw std::invalid_argument(
        "CollectDemonstrationServer: action_name must not be empty");
  }
  return loaded_config;
}

}  // namespace

CollectDemonstrationServer::CollectDemonstrationServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("collect_demonstration_server", options)
  , param_listener_(std::make_shared<rlc_frontend::ParamListener>(
        get_node_parameters_interface(), get_logger()))
  , cfg_(loadServerConfig(param_listener_->get_params()))
  , teleoperation_controller_(this, cfg_.teleoperation)
  , recorder_(std::make_unique<server_detail::JointDemonstrationRecorder>(
        this, cfg_.joint_states_topic, cfg_.state_stale_sec))
{
  action_server_ = rclcpp_action::create_server<CollectDemonstration>(
      this, cfg_.action_name,
      [this](const rclcpp_action::GoalUUID& uuid,
             const std::shared_ptr<const CollectDemonstration::Goal>& goal) {
        return handleGoal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle) {
        return handleCancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle) {
        handleAccepted(goal_handle);
      });

  RCLCPP_INFO(get_logger(), "CollectDemonstration action ready on '%s'",
              cfg_.action_name.c_str());
}

CollectDemonstrationServer::~CollectDemonstrationServer() = default;

rclcpp_action::GoalResponse CollectDemonstrationServer::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    const std::shared_ptr<const CollectDemonstration::Goal>& goal)
{
  (void)uuid;
  if (!goal || !std::isfinite(goal->sample_dt) || goal->sample_dt <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "rejecting invalid CollectDemonstration goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool expected = false;
  if (!active_goal_.compare_exchange_strong(expected, true))
  {
    RCLCPP_WARN(get_logger(), "rejecting concurrent CollectDemonstration goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CollectDemonstrationServer::handleCancel(
    const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(get_logger(), "cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CollectDemonstrationServer::handleAccepted(
    const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle)
{
  std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach();
}

void CollectDemonstrationServer::execute(
    const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle)
{
  const auto started_at = std::chrono::steady_clock::now();
  const double sample_dt = goal_handle->get_goal()->sample_dt;
  const auto sample_period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(sample_dt));

  try
  {
    const ServoPauseResult start_result = teleoperation_controller_.setActive(true);
    if (!start_result.success)
    {
      abortGoal(goal_handle, "failed to start teleoperation: " + start_result.message,
                elapsedSec(started_at), recorder_->sampleCount());
      return;
    }

    if (const auto error = recorder_->startRecording())
    {
      abortGoal(goal_handle, *error, elapsedSec(started_at), recorder_->sampleCount());
      return;
    }

    auto next_sample_time = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        cancelGoal(goal_handle, elapsedSec(started_at), recorder_->sampleCount());
        return;
      }

      if (const auto servo_status = teleoperation_controller_.status();
          servo_status && isHardServoHalt(servo_status->status.code))
      {
        abortGoal(goal_handle, makeServoHaltMessage(servo_status->status),
                  elapsedSec(started_at), recorder_->sampleCount());
        return;
      }

      if (const auto error = recorder_->recordSample())
      {
        abortGoal(goal_handle, *error, elapsedSec(started_at), recorder_->sampleCount());
        return;
      }

      publishFeedback(goal_handle, "collecting demonstration", elapsedSec(started_at),
                      recorder_->sampleCount());

      const DemonstrationRequest request = teleoperation_controller_.takeRequest();
      if (request == DemonstrationRequest::kAbort)
      {
        abortGoal(goal_handle, "demonstration collection aborted by operator",
                  elapsedSec(started_at), recorder_->sampleCount());
        return;
      }
      if (request == DemonstrationRequest::kFinish)
      {
        if (recorder_->sampleCount() < 2U)
        {
          abortGoal(goal_handle, "demonstration must contain at least two samples",
                    elapsedSec(started_at), recorder_->sampleCount());
          return;
        }

        succeedGoal(goal_handle, recorder_->buildTrajectory(), elapsedSec(started_at),
                    recorder_->sampleCount());
        return;
      }

      next_sample_time += sample_period;
      std::this_thread::sleep_until(next_sample_time);
    }

    cancelGoal(goal_handle, elapsedSec(started_at), recorder_->sampleCount());
  }
  catch (const std::exception& e)
  {
    abortGoal(goal_handle, e.what(), elapsedSec(started_at), recorder_->sampleCount());
  }
}

void CollectDemonstrationServer::publishFeedback(
    const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
    const std::string& feedback_msg, double elapsed_sec, std::size_t sample_count)
{
  auto feedback = std::make_shared<CollectDemonstration::Feedback>();
  feedback->feedback = feedback_msg;
  feedback->elapsed_sec = elapsed_sec;
  feedback->sample_count = static_cast<std::uint32_t>(sample_count);
  goal_handle->publish_feedback(feedback);
}

void CollectDemonstrationServer::succeedGoal(
    const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
    const moveit_msgs::msg::RobotTrajectory& demonstration, double elapsed_sec,
    std::size_t sample_count)
{
  requestTeleoperationStopped();

  auto result = std::make_shared<CollectDemonstration::Result>();
  result->demonstration = demonstration;
  result->elapsed_sec = elapsed_sec;
  result->sample_count = static_cast<std::uint32_t>(sample_count);
  goal_handle->succeed(result);
  active_goal_.store(false);

  RCLCPP_INFO(get_logger(), "SUCCESS (%.3fs, samples=%zu)", elapsed_sec, sample_count);
}

void CollectDemonstrationServer::abortGoal(
    const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
    const std::string& error, double elapsed_sec, std::size_t sample_count)
{
  requestTeleoperationStopped();

  auto result = std::make_shared<CollectDemonstration::Result>();
  result->error = error;
  result->elapsed_sec = elapsed_sec;
  result->sample_count = static_cast<std::uint32_t>(sample_count);
  goal_handle->abort(result);
  active_goal_.store(false);

  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
}

void CollectDemonstrationServer::cancelGoal(
    const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
    double elapsed_sec, std::size_t sample_count)
{
  requestTeleoperationStopped();

  auto result = std::make_shared<CollectDemonstration::Result>();
  result->error = "demonstration collection canceled";
  result->elapsed_sec = elapsed_sec;
  result->sample_count = static_cast<std::uint32_t>(sample_count);
  goal_handle->canceled(result);
  active_goal_.store(false);

  RCLCPP_WARN(get_logger(), "CANCELED (%.3fs, samples=%zu)", elapsed_sec, sample_count);
}

void CollectDemonstrationServer::requestTeleoperationStopped()
{
  const ServoPauseResult stop_result = teleoperation_controller_.setActive(false);
  if (!stop_result.success)
  {
    RCLCPP_WARN(get_logger(), "failed to stop teleoperation: %s",
                stop_result.message.c_str());
  }
}

}  // namespace rlc_frontend
