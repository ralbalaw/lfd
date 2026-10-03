#include "rlc_executive/executive/tree_runner.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include <behaviortree_cpp/basic_types.h>

#include "rlc_executive/config/executive_config.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"
#include "rlc_executive/executive/bt_node_registration.hpp"

namespace rlc_executive
{
TreeRunner::TreeRunner(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger,
                       std::string tree_xml_path, const ExecutiveConfig& config,
                       const PlanningConfigRegistry& planning_configs,
                       RobotResources& robot_resources, TickMode tick_mode,
                       std::chrono::milliseconds sleep_time)
  : tree_xml_path_(std::move(tree_xml_path)), opt_{ tick_mode, sleep_time }
{
  if (opt_.sleep_time.count() < 0)
  {
    throw std::invalid_argument("TreeRunner: sleep_time must be non-negative");
  }

  registerExecutiveBtNodes(factory_, node, logger, config, planning_configs,
                           robot_resources);
  loadTreeFromXmlFile();
}

void TreeRunner::loadTreeFromXmlFile()
{
  tree_ = factory_.createTreeFromFile(tree_xml_path_);
}

BT::NodeStatus TreeRunner::tick()
{
  switch (opt_.mode)
  {
    case TickMode::kOnceUnlessWokenUp:
      return tickOnce();
    case TickMode::kWhileRunning:
      return tickWhileRunning();
    case TickMode::kExactlyOnce:
      return tickExactlyOnce();
    default:
      throw std::runtime_error("TreeRunner: invalid tick mode");
  }
}

BT::NodeStatus TreeRunner::tickOnce()
{
  return tree_.tickOnce();
}

BT::NodeStatus TreeRunner::tickExactlyOnce()
{
  return tree_.tickExactlyOnce();
}

BT::NodeStatus TreeRunner::tickWhileRunning()
{
  return tree_.tickWhileRunning(opt_.sleep_time);
}

void TreeRunner::haltTree()
{
  tree_.haltTree();
}

void TreeRunner::reset()
{
  if (tree_xml_path_.empty())
  {
    throw std::runtime_error("TreeRunner::reset: no tree loaded yet");
  }

  haltTree();
  tree_ = factory_.createTreeFromFile(tree_xml_path_);
}

}  // namespace rlc_executive
