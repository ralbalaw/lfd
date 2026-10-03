#pragma once

#include <string>

namespace rlc_frontend
{

/// @brief MoveIt Servo service and status endpoints.
struct ServoConfig
{
  std::string pause_service_name = "/servo_node/pause_servo";
  std::string switch_command_type_service_name = "/servo_node/switch_command_type";
  std::string status_topic = "/servo_node/status";
};

/// @brief Joystick tokens for one directional control group.
struct ServoJoyAxisConfig
{
  std::string x = "LEFTX";
  std::string y = "LEFTY";
  std::string z_pos = "TRIGGERRIGHT";
  std::string z_neg = "TRIGGERLEFT";
};

/// @brief Joystick buttons that finish or abort demonstration collection.
struct ServoJoyButtonsConfig
{
  std::string finish = "START";
  std::string abort = "BACK";
};

/// @brief Joystick-driven Servo frontend settings.
struct ServoJoyFrontendConfig
{
  std::string joy_topic = "/joy";
  std::string twist_topic = "/servo_node/delta_twist_cmds";
  std::string joint_topic = "/servo_node/delta_joint_cmds";
  std::string pose_topic = "/servo_node/pose_target_cmds";
  std::string command_frame;

  double deadzone = 0.10;
  double linear_scale = 0.25;
  double angular_scale = 0.75;

  ServoJoyAxisConfig axis_linear{ "LEFTY", "LEFTX", "TRIGGERRIGHT", "TRIGGERLEFT" };
  ServoJoyAxisConfig axis_angular{ "RIGHTY", "RIGHTX", "RIGHTSHOULDER", "LEFTSHOULDER" };
  ServoJoyButtonsConfig buttons;
};

/// @brief Top-level teleoperation controller settings.
struct TeleoperationConfig
{
  std::string frontend = "joy";
  ServoConfig servo;
  ServoJoyFrontendConfig joy;
};

/// @brief Demonstration collection action server settings.
struct CollectDemonstrationConfig
{
  std::string action_name = "/collect_demonstration";
  std::string joint_states_topic = "/joint_states";
  double state_stale_sec = 0.2;
  TeleoperationConfig teleoperation;
};

}  // namespace rlc_frontend
