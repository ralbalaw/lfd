#pragma once

#include <atomic>
#include <string>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "rlc_frontend/frontend/frontend_config.hpp"
#include "rlc_frontend/frontend/teleoperation_frontend.hpp"
#include "rlc_frontend/frontend/teleoperation_session.hpp"
#include "rlc_frontend/frontend/types.hpp"

namespace rlc_frontend
{

/**
 * @brief Joystick teleoperation frontend that publishes Servo command topics.
 *
 * @details
 * This frontend converts joystick input into Servo command messages and latches
 * finish or abort requests into the shared `TeleoperationSession`.
 */
class ServoJoyFrontend final : public TeleoperationFrontend
{
public:
  /// @brief Servo command message type published from joystick input.
  enum class CommandMode
  {
    kTwist,
    kJointJog,
    kPose
  };

  /**
   * @brief Constructs the joystick teleoperation frontend.
   * @param[in] node ROS node used to create publishers and the joystick subscription.
   * @param[in] cfg Joystick frontend topic names and axis/button mappings.
   * @param[in] teleop_session Shared teleoperation session that receives demo requests.
   * The session reference is stored and must outlive this frontend.
   * @throws std::invalid_argument If a topic, frame, scale, or joystick mapping is
   * invalid.
   */
  explicit ServoJoyFrontend(rclcpp::Node* node, const ServoJoyFrontendConfig& cfg,
                            TeleoperationSession& teleop_session);

  /**
   * @brief Returns whether joystick input currently drives Servo teleoperation.
   * @return `true` when joystick input is allowed to publish Servo commands.
   */
  bool isEnabled() const noexcept override
  {
    return enabled_.load(std::memory_order_relaxed);
  }

  /**
   * @brief Enables or disables joystick command publication.
   * @param[in] enabled `true` to publish Servo commands from joystick input.
   */
  void setEnabled(bool enabled) noexcept override
  {
    enabled_.store(enabled, std::memory_order_relaxed);
  }

  /**
   * @brief Returns the current joystick command output mode.
   * @return Servo command message type published for joystick input.
   */
  CommandMode commandMode() const noexcept
  {
    return mode_.load(std::memory_order_relaxed);
  }

  /**
   * @brief Selects the joystick command output mode.
   * @param[in] mode Servo command message type to publish for joystick input.
   */
  void setCommandMode(CommandMode mode) noexcept
  {
    mode_.store(mode, std::memory_order_relaxed);
  }

private:
  struct ResolvedAxisConfig
  {
    int x = 0;
    int y = 0;
    int z_pos = 0;
    int z_neg = 0;
  };

  struct ResolvedButtonConfig
  {
    int finish = 0;
    int abort = 0;
  };

  void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg);

  void publishServoCommand(const sensor_msgs::msg::Joy& joy);

  geometry_msgs::msg::TwistStamped
  makeTwistCommand(const sensor_msgs::msg::Joy& joy) const;
  control_msgs::msg::JointJog makeJointJogCommand(const sensor_msgs::msg::Joy& joy) const;
  geometry_msgs::msg::PoseStamped makePoseCommand(const sensor_msgs::msg::Joy& joy) const;

  double axisValue(const sensor_msgs::msg::Joy& joy, int index) const noexcept;
  double triggerAxisValue(const sensor_msgs::msg::Joy& joy, int pos_index,
                          int neg_index) const noexcept;

  double buttonPairValue(const sensor_msgs::msg::Joy& joy, int pos_index,
                         int neg_index) const noexcept;
  bool isButtonPressed(const sensor_msgs::msg::Joy& joy, int index) const noexcept;

  std::optional<DemonstrationRequest>
  detectDemonstrationRequest(const sensor_msgs::msg::Joy& joy);

private:
  rclcpp::Node* node_ = nullptr;
  ServoJoyFrontendConfig cfg_;
  TeleoperationSession& teleop_session_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::atomic_bool enabled_{ false };
  std::atomic<CommandMode> mode_{ CommandMode::kTwist };

  ResolvedAxisConfig axis_linear_;
  ResolvedAxisConfig axis_angular_;
  ResolvedButtonConfig buttons_;
  std::string command_frame_;
  bool finish_button_prev_ = false;
  bool abort_button_prev_ = false;
};

}  // namespace rlc_frontend
