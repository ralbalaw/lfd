#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rlc_frontend::server_detail
{

class JointDemonstrationRecorder final
{
public:
  JointDemonstrationRecorder(rclcpp::Node* node, const std::string& joint_states_topic,
                             double state_stale_sec);

  [[nodiscard]] std::optional<std::string> startRecording();
  [[nodiscard]] std::optional<std::string> recordSample();
  [[nodiscard]] std::size_t sampleCount() const noexcept;
  [[nodiscard]] moveit_msgs::msg::RobotTrajectory buildTrajectory() const;

private:
  struct RecordedSample
  {
    std::vector<double> positions;
    std::chrono::nanoseconds time_from_start;
  };

  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);
  [[nodiscard]] std::optional<sensor_msgs::msg::JointState> latestFreshJointState() const;
  void resetRecording();

  rclcpp::Node* node_;
  double state_stale_sec_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  mutable std::mutex joint_state_mtx_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;

  bool recording_started_{ false };
  std::vector<std::string> joint_names_;
  std::size_t joint_dim_{ 0 };
  std::optional<std::chrono::steady_clock::time_point> first_sample_at_;
  std::vector<RecordedSample> samples_;
};

}  // namespace rlc_frontend::server_detail
