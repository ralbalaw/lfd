#pragma once

#include <chrono>
#include <string>

#include <rclcpp/qos.hpp>

namespace rlc_executive
{

/**
 * @brief Selects the BehaviorTree.CPP tick method used by `TreeRunner`.
 */
enum class TickMode
{
  kOnceUnlessWokenUp,
  kWhileRunning,
  kExactlyOnce
};

/**
 * @brief Configures BehaviorTree execution.
 *
 * @details
 * - `tick_mode`: BehaviorTree.CPP tick method used by `TreeRunner`.
 * - `tick_rate_hz`: ROS timer rate used by the executive.
 * - `sleep_time`: Maximum block time [ms] used by `tickWhileRunning()`.
 */
struct BehaviorTreeConfig
{
  TickMode tick_mode{ TickMode::kOnceUnlessWokenUp };
  double tick_rate_hz{ 30.0 };
  std::chrono::milliseconds sleep_time{ 100 };
};

/**
 * @brief Configures robot-state subscriptions and TF lookups.
 *
 * @details
 * - `joint_states_topic`: Source topic carrying `sensor_msgs::msg::JointState`.
 * - `base_frame`: Base frame used for end-effector transform lookups.
 * - `ee_frame`: End-effector frame used for transform lookups.
 * - `tf_timeout_sec`: TF lookup timeout [s].
 * - `state_stale_sec`: Maximum accepted joint-state age [s].
 * - `joint_state_qos`: QoS profile used for the joint-state subscription.
 */
struct StateConfig
{
  std::string joint_states_topic = "/joint_states";
  std::string base_frame = "base_link";
  std::string ee_frame = "tool0";
  double tf_timeout_sec = 0.05;
  double state_stale_sec = 0.2;
  rclcpp::QoS joint_state_qos = rclcpp::SensorDataQoS();
};

/**
 * @brief Configures MoveGroup planning endpoints.
 *
 * @details
 * - `action_name`: Default MoveGroup action name used by ROS-aware BT planning nodes.
 * - `get_planning_scene_service_name`: Planning-scene service name reserved for future use.
 */
struct MoveGroupConfig
{
  std::string action_name = "/move_action";
  std::string get_planning_scene_service_name = "/get_planning_scene";
};

/**
 * @brief Configures trajectory execution endpoints.
 *
 * @details
 * - `execute_traj_action_name`: Default action name injected into
 *   `ExecuteTrajectory`.
 */
struct ExecutionConfig
{
  std::string execute_traj_action_name = "/execute_trajectory";
};

/**
 * @brief Configures planning-config selection.
 *
 * @details
 * - `default_config`: Default config name resolved through the planning config map.
 */
struct PlanningSelectionConfig
{
  std::string default_config;
};

/**
 * @brief Configures the external demonstration collection action endpoint.
 *
 * @details
 * - `action_name`: Default `CollectDemonstration` action name injected into
 *   `RecordServoDemo`.
 */
struct DemonstrationCollectionConfig
{
  std::string action_name = "/collect_demonstration";
};

/**
 * @brief Configures the executive's environment-monitor connection.
 *
 * @details
 * - `monitor_namespace`: ROS namespace hosting the monitor endpoints.
 * - `env_name`: Required Tesseract environment name forwarded to the monitor.
 * - `wait_timeout`: Maximum wait time [ms] for the monitor interface to become ready.
 */
struct MonitorConfig
{
  std::string monitor_namespace = "tesseract_monitor";
  std::string env_name = "default";
  std::chrono::milliseconds wait_timeout{ 3000 };
};

/**
 * @brief Root runtime configuration for the executive.
 *
 * @details
 * This object owns the typed configuration loaded at startup and groups settings by
 * subsystem so each component can consume only its relevant section.
 */
struct ExecutiveConfig
{
  BehaviorTreeConfig behavior_tree;
  StateConfig state;
  MoveGroupConfig move_group;
  ExecutionConfig execution;
  PlanningSelectionConfig planning;
  DemonstrationCollectionConfig demonstration_collection;
  MonitorConfig monitor;
};

}  // namespace rlc_executive
