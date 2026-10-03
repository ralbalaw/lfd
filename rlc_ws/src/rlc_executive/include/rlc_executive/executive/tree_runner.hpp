#pragma once

#include <chrono>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include "rlc_executive/config/executive_config.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"
#include "rlc_executive/executive/robot_resources.hpp"

namespace rlc_executive
{

/**
 * @brief Registers executive BT nodes, owns a BehaviorTree factory, and ticks one tree.
 *
 * @details
 * `TreeRunner` borrows ROS/runtime resources passed at construction. Those borrowed
 * objects must outlive the runner and every tree node created by its factory.
 */
class TreeRunner final
{
  /**
   * @brief Configures tick behavior owned by `TreeRunner`.
   *
   * @details
   * - `mode`: BehaviorTree.CPP tick method used by `TreeRunner::tick()`.
   * - `sleep_time`: Maximum blocking time [ms] passed to
   * `BT::Tree::tickWhileRunning()` when `mode == TickMode::kWhileRunning`.
   */
  struct Options
  {
    TickMode mode{ TickMode::kOnceUnlessWokenUp };
    std::chrono::milliseconds sleep_time{ 100 };
  };

public:
  /**
   * @brief Constructs a runner, registers built-in BT nodes, and loads a tree.
   * @param[in] node ROS node retained by ROS-aware BT nodes; must not be nullptr
   * and must outlive all created BT nodes.
   * @param[in] logger Logger used by nodes that need explicit logging.
   * @param[in] tree_xml_path Path to the BT XML file.
   * @param[in] config Executive config used during BT node registration.
   * @param[in] planning_configs Planning config registry forwarded to BT node
   * registrations; must outlive all created BT nodes.
   * @param[in,out] robot_resources Robot resources forwarded to BT node
   * registrations; must outlive all created BT nodes.
   * @param[in] tick_mode BehaviorTree.CPP tick method used by tick().
   * @param[in] sleep_time Maximum block time [ms] used by tickWhileRunning().
   * @throws std::invalid_argument If `sleep_time` is negative.
   * @throws std::runtime_error If the tree cannot be loaded.
   */
  TreeRunner(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger,
             std::string tree_xml_path, const ExecutiveConfig& config,
             const PlanningConfigRegistry& planning_configs,
             RobotResources& robot_resources, TickMode tick_mode,
             std::chrono::milliseconds sleep_time);

  /**
   * @brief Loads the configured XML tree path into the BehaviorTree factory.
   * @throws std::runtime_error If BehaviorTree.CPP cannot parse or instantiate the
   * tree.
   */
  void loadTreeFromXmlFile();

  /**
   * @brief Ticks the tree using the configured tick mode.
   * @return BehaviorTree status after ticking.
   * @throws std::runtime_error If the configured tick mode is invalid.
   */
  BT::NodeStatus tick();

  /**
   * @brief Ticks the tree exactly once.
   * @return BehaviorTree status after one exact tick.
   */
  BT::NodeStatus tickExactlyOnce();

  /**
   * @brief Ticks the tree once unless it is woken up by BehaviorTree.CPP.
   * @return BehaviorTree status after one wake-aware tick.
   */
  BT::NodeStatus tickOnce();

  /**
   * @brief Ticks while the tree is running using the configured maximum block time.
   * @return BehaviorTree status after ticking.
   */
  BT::NodeStatus tickWhileRunning();

  /// @brief Halts the loaded tree.
  void haltTree();

  /**
   * @brief Halts the current tree and reloads it from the configured XML path.
   * @throws std::runtime_error If no XML path is configured.
   */
  void reset();

  /**
   * @brief Returns the currently loaded behavior tree.
   *
   * @return Reference to the tree instance.
   *
   * @note This reference is valid after loadTreeFromXmlFile() has been called and
   * remains valid until the tree is reloaded or reset.
   */
  const BT::Tree& tree() const
  {
    return tree_;
  }

private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  std::string tree_xml_path_;
  Options opt_;
};

}  // namespace rlc_executive
