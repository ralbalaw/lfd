#include "recording/joint_demonstration_recorder.hpp"

#include <cmath>
#include <stdexcept>

#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>

namespace rlc_frontend::server_detail
{

JointDemonstrationRecorder::JointDemonstrationRecorder(
    rclcpp::Node* node, const std::string& joint_states_topic, double state_stale_sec)
  : node_(node), state_stale_sec_(state_stale_sec)
{
  if (node_ == nullptr)
  {
    throw std::invalid_argument("JointDemonstrationRecorder: node must not be null");
  }
  if (joint_states_topic.empty())
  {
    throw std::invalid_argument(
        "JointDemonstrationRecorder: joint_states_topic must not be empty");
  }
  if (!std::isfinite(state_stale_sec_) || state_stale_sec_ < 0.0)
  {
    throw std::invalid_argument(
        "JointDemonstrationRecorder: state_stale_sec must be finite and >= 0");
  }

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        onJointState(msg);
      });
}

std::optional<std::string> JointDemonstrationRecorder::startRecording()
{
  resetRecording();

  const auto joint_state = latestFreshJointState();
  if (!joint_state || joint_state->position.empty())
  {
    return "latest joint state is unavailable or stale";
  }
  if (!joint_state->name.empty() &&
      joint_state->name.size() != joint_state->position.size())
  {
    return "joint state name dimension does not match positions";
  }

  joint_names_ = joint_state->name;
  joint_dim_ = joint_state->position.size();
  recording_started_ = true;
  return std::nullopt;
}

std::optional<std::string> JointDemonstrationRecorder::recordSample()
{
  if (!recording_started_)
  {
    return "joint demonstration recording has not started";
  }

  const auto joint_state = latestFreshJointState();
  if (!joint_state)
  {
    return "latest joint state is unavailable or stale while recording";
  }
  if (joint_state->position.size() != joint_dim_)
  {
    return "joint state dimension changed while recording";
  }
  if (joint_state->name != joint_names_)
  {
    return "joint state names changed while recording";
  }

  const auto sampled_at = std::chrono::steady_clock::now();
  if (!first_sample_at_)
  {
    first_sample_at_ = sampled_at;
  }

  auto time_from_start = std::chrono::duration_cast<std::chrono::nanoseconds>(
      sampled_at - *first_sample_at_);
  if (!samples_.empty() && time_from_start <= samples_.back().time_from_start)
  {
    time_from_start = samples_.back().time_from_start + std::chrono::nanoseconds(1);
  }

  samples_.push_back({ joint_state->position, time_from_start });
  return std::nullopt;
}

std::size_t JointDemonstrationRecorder::sampleCount() const noexcept
{
  return samples_.size();
}

moveit_msgs::msg::RobotTrajectory JointDemonstrationRecorder::buildTrajectory() const
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = joint_names_;
  trajectory.joint_trajectory.points.resize(samples_.size());

  for (std::size_t row = 0; row < samples_.size(); ++row)
  {
    auto& point = trajectory.joint_trajectory.points[row];
    point.positions = samples_[row].positions;
    point.time_from_start = rclcpp::Duration(samples_[row].time_from_start);
  }

  return trajectory;
}

void JointDemonstrationRecorder::onJointState(
    const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mtx_);
  latest_joint_state_ = msg;
}

std::optional<sensor_msgs::msg::JointState>
JointDemonstrationRecorder::latestFreshJointState() const
{
  sensor_msgs::msg::JointState::ConstSharedPtr joint_state;
  {
    std::lock_guard<std::mutex> lock(joint_state_mtx_);
    joint_state = latest_joint_state_;
  }

  if (!joint_state)
  {
    return std::nullopt;
  }

  const rclcpp::Time stamp(joint_state->header.stamp);
  if (stamp.nanoseconds() != 0)
  {
    const double age_sec = (node_->get_clock()->now() - stamp).seconds();
    if (age_sec > state_stale_sec_)
    {
      return std::nullopt;
    }
  }

  return *joint_state;
}

void JointDemonstrationRecorder::resetRecording()
{
  recording_started_ = false;
  joint_names_.clear();
  joint_dim_ = 0;
  first_sample_at_.reset();
  samples_.clear();
}

}  // namespace rlc_frontend::server_detail
