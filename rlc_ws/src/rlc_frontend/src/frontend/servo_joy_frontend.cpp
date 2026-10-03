#include "rlc_frontend/frontend/servo_joy_frontend.hpp"

#include <cmath>
#include <stdexcept>

#include "rlc_frontend/frontend/joy_mapping.hpp"

namespace rlc_frontend
{

ServoJoyFrontend::ServoJoyFrontend(rclcpp::Node* node, const ServoJoyFrontendConfig& cfg,
                                   TeleoperationSession& teleop_session)
  : node_(node), cfg_(cfg), teleop_session_(teleop_session)
{
  if (cfg_.joy_topic.empty())
  {
    throw std::invalid_argument("ServoJoyFrontend: joy_topic must not be empty");
  }
  if (cfg_.twist_topic.empty())
  {
    throw std::invalid_argument("ServoJoyFrontend: twist_topic must not be empty");
  }
  if (cfg_.joint_topic.empty())
  {
    throw std::invalid_argument("ServoJoyFrontend: joint_topic must not be empty");
  }
  if (cfg_.pose_topic.empty())
  {
    throw std::invalid_argument("ServoJoyFrontend: pose_topic must not be empty");
  }
  if (cfg_.command_frame.empty())
  {
    throw std::invalid_argument("ServoJoyFrontend: command_frame must not be empty");
  }
  if (cfg_.deadzone < 0.0)
  {
    throw std::invalid_argument("ServoJoyFrontend: deadzone must be >= 0");
  }
  if (cfg_.linear_scale < 0.0)
  {
    throw std::invalid_argument("ServoJoyFrontend: linear_scale must be >= 0");
  }
  if (cfg_.angular_scale < 0.0)
  {
    throw std::invalid_argument("ServoJoyFrontend: angular_scale must be >= 0");
  }

  command_frame_ = cfg_.command_frame;
  axis_linear_.x = frontend::resolveJoyAxisIndex(cfg_.axis_linear.x, "axis_linear.x");
  axis_linear_.y = frontend::resolveJoyAxisIndex(cfg_.axis_linear.y, "axis_linear.y");
  axis_linear_.z_pos =
      frontend::resolveJoyAxisIndex(cfg_.axis_linear.z_pos, "axis_linear.z_pos");
  axis_linear_.z_neg =
      frontend::resolveJoyAxisIndex(cfg_.axis_linear.z_neg, "axis_linear.z_neg");

  axis_angular_.x = frontend::resolveJoyAxisIndex(cfg_.axis_angular.x, "axis_angular.x");
  axis_angular_.y = frontend::resolveJoyAxisIndex(cfg_.axis_angular.y, "axis_angular.y");
  axis_angular_.z_pos =
      frontend::resolveJoyButtonIndex(cfg_.axis_angular.z_pos, "axis_angular.z_pos");
  axis_angular_.z_neg =
      frontend::resolveJoyButtonIndex(cfg_.axis_angular.z_neg, "axis_angular.z_neg");

  buttons_.finish =
      frontend::resolveJoyButtonIndex(cfg_.buttons.finish, "buttons.finish");
  buttons_.abort = frontend::resolveJoyButtonIndex(cfg_.buttons.abort, "buttons.abort");

  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(cfg_.twist_topic,
                                                                         rclcpp::QoS(10));

  joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(cfg_.joint_topic,
                                                                    rclcpp::QoS(10));

  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(cfg_.pose_topic,
                                                                       rclcpp::QoS(10));

  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      cfg_.joy_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { this->onJoy(msg); });
}

void ServoJoyFrontend::onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
{
  if (!msg)
  {
    return;
  }

  if (!isEnabled())
  {
    return;
  }

  if (const auto request = detectDemonstrationRequest(*msg); request.has_value())
  {
    teleop_session_.setRequest(*request);
    return;
  }

  publishServoCommand(*msg);
}

void ServoJoyFrontend::publishServoCommand(const sensor_msgs::msg::Joy& joy)
{
  switch (commandMode())
  {
    case CommandMode::kTwist:
    {
      twist_pub_->publish(makeTwistCommand(joy));
      return;
    }

    case CommandMode::kJointJog:
    case CommandMode::kPose:
    {
      return;
    }
  }
}

geometry_msgs::msg::TwistStamped
ServoJoyFrontend::makeTwistCommand(const sensor_msgs::msg::Joy& joy) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = command_frame_;

  cmd.twist.linear.x = cfg_.linear_scale * axisValue(joy, axis_linear_.x);
  cmd.twist.linear.y = cfg_.linear_scale * axisValue(joy, axis_linear_.y);
  cmd.twist.linear.z =
      cfg_.linear_scale * triggerAxisValue(joy, axis_linear_.z_pos, axis_linear_.z_neg);

  cmd.twist.angular.x = cfg_.angular_scale * axisValue(joy, axis_angular_.x);
  cmd.twist.angular.y = cfg_.angular_scale * axisValue(joy, axis_angular_.y);
  cmd.twist.angular.z =
      cfg_.angular_scale * buttonPairValue(joy, axis_angular_.z_pos, axis_angular_.z_neg);

  return cmd;
}

double ServoJoyFrontend::axisValue(const sensor_msgs::msg::Joy& joy,
                                   int index) const noexcept
{
  const auto i = static_cast<std::size_t>(index);
  const double value = static_cast<double>(joy.axes[i]);

  if (std::abs(value) < cfg_.deadzone)
  {
    return 0.0;
  }

  return value;
}

bool ServoJoyFrontend::isButtonPressed(const sensor_msgs::msg::Joy& joy,
                                       int index) const noexcept
{
  const auto i = static_cast<std::size_t>(index);
  return joy.buttons[i] != 0;
}

double ServoJoyFrontend::triggerAxisValue(const sensor_msgs::msg::Joy& joy, int pos_index,
                                          int neg_index) const noexcept
{
  const auto normalize_trigger = [this, &joy](int index) noexcept -> double {
    const auto i = static_cast<std::size_t>(index);
    const double raw = static_cast<double>(joy.axes[i]);
    const double value = (raw < 0.0) ? 0.5 * (raw + 1.0) : raw;

    if (std::abs(value) < cfg_.deadzone)
    {
      return 0.0;
    }

    return value;
  };

  return normalize_trigger(pos_index) - normalize_trigger(neg_index);
}

double ServoJoyFrontend::buttonPairValue(const sensor_msgs::msg::Joy& joy, int pos_index,
                                         int neg_index) const noexcept
{
  const bool pos = isButtonPressed(joy, pos_index);
  const bool neg = isButtonPressed(joy, neg_index);

  if (pos == neg)
  {
    return 0.0;
  }

  return pos ? 1.0 : -1.0;
}

std::optional<DemonstrationRequest>
ServoJoyFrontend::detectDemonstrationRequest(const sensor_msgs::msg::Joy& joy)
{
  const bool finish_pressed = isButtonPressed(joy, buttons_.finish);
  const bool abort_pressed = isButtonPressed(joy, buttons_.abort);

  std::optional<DemonstrationRequest> out;

  if (finish_pressed && !finish_button_prev_)
  {
    out = DemonstrationRequest::kFinish;
  }
  else if (abort_pressed && !abort_button_prev_)
  {
    out = DemonstrationRequest::kAbort;
  }

  finish_button_prev_ = finish_pressed;
  abort_button_prev_ = abort_pressed;

  return out;
}

}  // namespace rlc_frontend
