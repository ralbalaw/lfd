
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include "rlc_executive/config/executive_config.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"
#include "rlc_executive/executive/robot_resources.hpp"

namespace rlc_executive
{

/**
 * @brief Registers built-in executive BT node types with a BehaviorTree factory.
 * @param[in,out] factory BehaviorTree factory receiving node registrations.
 * @param[in] node ROS node retained by ROS-aware BT nodes; must outlive created BT
 * nodes.
 * @param[in] logger Logger forwarded to BT nodes that need explicit diagnostics.
 * @param[in] config Executive runtime config used for default action names and
 * monitor settings.
 * @param[in] planning_configs Planning config registry forwarded to planning BT
 * nodes; must outlive created BT nodes.
 * @param[in,out] robot_resources Robot resources forwarded to BT nodes; must
 * outlive created BT nodes.
 */
void registerExecutiveBtNodes(BT::BehaviorTreeFactory& factory,
                              const rclcpp::Node::SharedPtr& node,
                              const rclcpp::Logger& logger, const ExecutiveConfig& config,
                              const PlanningConfigRegistry& planning_configs,
                              RobotResources& robot_resources);
}  // namespace rlc_executive
