#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt_executive_parameters.hpp"
#include "rlc_executive/config/executive_config.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"
#include "rlc_executive/executive/robot_resources.hpp"
#include "rlc_executive/executive/tree_runner.hpp"

namespace rlc_executive
{

/**
 * @brief Loads and ticks a BehaviorTree executive using a retained ROS 2 node.
 *
 * @details
 * Parameters:
 * - Executive settings are loaded through generated `ParamListener` parameters.
 * - `planning_configs_path`: Path to planning configs YAML config.
 * - `default_tree`: Path to BT XML to load.
 * The executive owns runtime services and passes non-owning pointers to BT nodes.
 */
class BtExecutive final
{
public:
  /// @brief Callback invoked once when the tree reaches a terminal status.
  using CompletionCallback = std::function<void(BT::NodeStatus)>;

  /**
   * @brief Constructs the BehaviorTree executive wrapper.
   * @param[in] node Shared ROS node used for parameters, timers, logging, and ROS BT
   * clients; must not be nullptr and must outlive this object.
   * @param[in] completion_callback Callback invoked once when the tree completes;
   * must not be empty.
   * @throws std::invalid_argument If `node` is nullptr or `completion_callback` is
   * empty.
   */
  BtExecutive(rclcpp::Node::SharedPtr node, CompletionCallback completion_callback);

  /// @brief Loads configs, creates the BT, and starts ticking.
  void start();

private:
  struct StartupPaths
  {
    std::string planning_configs_path;
    std::string tree_xml_path;
  };

  StartupPaths loadStartupPaths() const;
  void loadConfiguration(const StartupPaths& paths);
  RobotResources::Options makeRobotResourcesOptions() const;
  void maybeStartGroot2Publisher();
  void startTickTimer();
  void tick();
  void onTreeCompleted(BT::NodeStatus status);

  rclcpp::Node::SharedPtr node_;
  CompletionCallback completion_callback_;
  rclcpp::CallbackGroup::SharedPtr tick_callback_group_;
  rclcpp::Logger logger_;

  std::shared_ptr<rlc_executive::ParamListener> param_listener_;
  rlc_executive::Params params_;

  ExecutiveConfig cfg_;
  std::optional<PlanningConfigRegistry> planning_configs_;
  std::optional<RobotResources> robot_resources_;

  std::optional<TreeRunner> tree_runner_;

  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  bool started_{ false };
  bool completed_{ false };
};

}  // namespace rlc_executive
