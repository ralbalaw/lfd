#include "rlc_frontend/frontend/frontend_config_parameters.hpp"

namespace rlc_frontend
{
namespace config
{

CollectDemonstrationConfig
loadCollectDemonstrationConfigFromParameters(const rlc_frontend::Params& params)
{
  CollectDemonstrationConfig config;

  config.action_name = params.action_name;
  config.joint_states_topic = params.joint_states_topic;
  config.state_stale_sec = params.state_stale_sec;

  config.teleoperation.frontend = params.teleoperation.frontend;
  config.teleoperation.servo.pause_service_name =
      params.teleoperation.servo.pause_service_name;
  config.teleoperation.servo.switch_command_type_service_name =
      params.teleoperation.servo.switch_command_type_service_name;
  config.teleoperation.servo.status_topic = params.teleoperation.servo.status_topic;

  config.teleoperation.joy.joy_topic = params.teleoperation.joy.joy_topic;
  config.teleoperation.joy.twist_topic = params.teleoperation.joy.twist_topic;
  config.teleoperation.joy.joint_topic = params.teleoperation.joy.joint_topic;
  config.teleoperation.joy.pose_topic = params.teleoperation.joy.pose_topic;
  config.teleoperation.joy.command_frame = params.teleoperation.joy.command_frame;
  config.teleoperation.joy.deadzone = params.teleoperation.joy.deadzone;
  config.teleoperation.joy.linear_scale = params.teleoperation.joy.linear_scale;
  config.teleoperation.joy.angular_scale = params.teleoperation.joy.angular_scale;

  config.teleoperation.joy.axis_linear.x = params.teleoperation.joy.axis_linear.x;
  config.teleoperation.joy.axis_linear.y = params.teleoperation.joy.axis_linear.y;
  config.teleoperation.joy.axis_linear.z_pos = params.teleoperation.joy.axis_linear.z_pos;
  config.teleoperation.joy.axis_linear.z_neg = params.teleoperation.joy.axis_linear.z_neg;

  config.teleoperation.joy.axis_angular.x = params.teleoperation.joy.axis_angular.x;
  config.teleoperation.joy.axis_angular.y = params.teleoperation.joy.axis_angular.y;
  config.teleoperation.joy.axis_angular.z_pos =
      params.teleoperation.joy.axis_angular.z_pos;
  config.teleoperation.joy.axis_angular.z_neg =
      params.teleoperation.joy.axis_angular.z_neg;

  config.teleoperation.joy.buttons.finish = params.teleoperation.joy.buttons.finish;
  config.teleoperation.joy.buttons.abort = params.teleoperation.joy.buttons.abort;

  return config;
}

}  // namespace config
}  // namespace rlc_frontend
