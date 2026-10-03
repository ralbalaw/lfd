#include <chrono>
#include <stdexcept>
#include <string>
#include <string_view>

#include "rlc_executive/config/config_loader.hpp"

namespace rlc_executive
{
namespace config
{
namespace
{

TickMode tickModeFromToken(const std::string& token)
{
  if (token == "ONCE_UNLESS_WOKEN_UP")
  {
    return TickMode::kOnceUnlessWokenUp;
  }
  if (token == "WHILE_RUNNING")
  {
    return TickMode::kWhileRunning;
  }
  if (token == "EXACTLY_ONCE")
  {
    return TickMode::kExactlyOnce;
  }

  throw std::invalid_argument(
      "behavior_tree.tick.mode must be one of: ONCE_UNLESS_WOKEN_UP, "
      "WHILE_RUNNING, EXACTLY_ONCE");
}

void requireNotEmpty(const std::string& value, std::string_view label)
{
  if (value.empty())
  {
    throw std::invalid_argument(std::string(label) + " must not be empty");
  }
}

void validateConfig(const ExecutiveConfig& config)
{
  if (config.behavior_tree.tick_rate_hz <= 0.0)
  {
    throw std::invalid_argument("behavior_tree.tick_rate_hz must be > 0");
  }

  requireNotEmpty(config.state.joint_states_topic, "state.joint_states_topic");
  requireNotEmpty(config.execution.execute_traj_action_name,
                  "execution.execute_traj_action_name");
  requireNotEmpty(config.demonstration_collection.action_name,
                  "demonstration_collection.action_name");
  requireNotEmpty(config.planning.default_config, "planning.default_config");
  requireNotEmpty(config.monitor.monitor_namespace, "monitor.monitor_namespace");
  requireNotEmpty(config.monitor.env_name, "monitor.env_name");

  if (config.monitor.wait_timeout.count() < 0)
  {
    throw std::invalid_argument("monitor.wait_timeout must be >= 0");
  }
}

}  // namespace

ExecutiveConfig loadExecutiveConfigFromParameters(const rlc_executive::Params& params)
{
  ExecutiveConfig config;

  config.behavior_tree.tick_rate_hz = params.behavior_tree.tick.rate_hz;
  config.behavior_tree.tick_mode = tickModeFromToken(params.behavior_tree.tick.mode);
  config.behavior_tree.sleep_time =
      std::chrono::milliseconds(params.behavior_tree.tick.while_running_max_block_ms);

  config.state.joint_states_topic = params.state.joint_states_topic;
  config.state.base_frame = params.state.base_frame;
  config.state.ee_frame = params.state.ee_frame;
  config.state.tf_timeout_sec = params.state.tf_timeout_sec;
  config.state.state_stale_sec = params.state.state_stale_sec;

  config.move_group.action_name = params.move_group.action_name;
  config.move_group.get_planning_scene_service_name =
      params.move_group.get_planning_scene_service_name;

  config.execution.execute_traj_action_name = params.execution.execute_traj_action_name;

  config.planning.default_config = params.planning.default_config;

  config.demonstration_collection.action_name =
      params.demonstration_collection.action_name;

  config.monitor.monitor_namespace = params.monitor.monitor_namespace;
  config.monitor.env_name = params.monitor.env_name;
  config.monitor.wait_timeout = std::chrono::milliseconds(params.monitor.wait_timeout_ms);

  validateConfig(config);

  return config;
}

}  // namespace config
}  // namespace rlc_executive
