#include "rlc_executive/executive/bt_executive.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/basic_types.h>

#include "rlc_executive/config/config_loader.hpp"
#include "rlc_executive/executive/non_null.hpp"

namespace rlc_executive
{
BtExecutive::BtExecutive(rclcpp::Node::SharedPtr node,
                         CompletionCallback completion_callback)
  : node_(requireNonNull(std::move(node), "BtExecutive: node"))
  , completion_callback_(std::move(completion_callback))
  , logger_(node_->get_logger())
  , param_listener_(std::make_shared<rlc_executive::ParamListener>(node_))
  , params_(param_listener_->get_params())
{
  if (!completion_callback_)
  {
    throw std::invalid_argument("BtExecutive: completion_callback must not be empty");
  }
}

void BtExecutive::start()
{
  if (started_)
  {
    throw std::runtime_error("BtExecutive: start() was called more than once");
  }
  started_ = true;

  const StartupPaths paths = loadStartupPaths();
  loadConfiguration(paths);
  robot_resources_ = RobotResources::create(node_, logger_, makeRobotResourcesOptions());
  tree_runner_.emplace(node_, logger_, paths.tree_xml_path, cfg_, *planning_configs_,
                       *robot_resources_, cfg_.behavior_tree.tick_mode,
                       cfg_.behavior_tree.sleep_time);
  maybeStartGroot2Publisher();
  startTickTimer();
}

BtExecutive::StartupPaths BtExecutive::loadStartupPaths() const
{
  const auto share = ament_index_cpp::get_package_share_directory("rlc_executive");

  auto make_path = [&](const std::string& s) -> std::string {
    if (std::filesystem::path{ s }.is_absolute())
    {
      return s;
    }
    return share + "/" + s;
  };

  StartupPaths paths{ make_path(params_.planning_configs_path),
                      make_path(params_.default_tree) };

  RCLCPP_INFO(logger_, "Starting BT executive");
  RCLCPP_INFO(logger_, "planning_configs_path: %s", paths.planning_configs_path.c_str());
  RCLCPP_INFO(logger_, "default_tree: %s", paths.tree_xml_path.c_str());

  return paths;
}

void BtExecutive::loadConfiguration(const StartupPaths& paths)
{
  cfg_ = config::loadExecutiveConfigFromParameters(params_);
  planning_configs_.emplace(config::loadPlanningConfigRegistryFromYaml(
      paths.planning_configs_path, cfg_.planning.default_config));

  RCLCPP_INFO(logger_, "tick_rate_hz: %.2f", cfg_.behavior_tree.tick_rate_hz);
}

RobotResources::Options BtExecutive::makeRobotResourcesOptions() const
{
  RobotResources::Options resources_options;
  resources_options.joint_states_topic = cfg_.state.joint_states_topic;
  resources_options.get_scene_service = cfg_.move_group.get_planning_scene_service_name;
  resources_options.monitor_namespace = cfg_.monitor.monitor_namespace;
  resources_options.env_name = cfg_.monitor.env_name;
  resources_options.monitor_wait_timeout = cfg_.monitor.wait_timeout;

  return resources_options;
}

void BtExecutive::maybeStartGroot2Publisher()
{
  if (params_.enable_groot2)
  {
    const unsigned port = static_cast<unsigned>(params_.groot2_port);
    try
    {
      groot2_publisher_ =
          std::make_unique<BT::Groot2Publisher>(tree_runner_->tree(), port);
      RCLCPP_INFO(logger_, "Groot2Publisher listening on tcp://0.0.0.0:%u", port);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_,
                   "Failed to start Groot2Publisher on port %u: %s (continuing "
                   "without Groot2)",
                   port, e.what());
    }
  }
}

void BtExecutive::startTickTimer()
{
  const auto period =
      std::chrono::duration<double>(1.0 / cfg_.behavior_tree.tick_rate_hz);

  const auto tick_callback = [this]() { tick(); };

  tick_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tick_timer_ = node_->create_wall_timer(period, tick_callback, tick_callback_group_);
}

void BtExecutive::tick()
{
  if (completed_)
  {
    return;
  }

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  try
  {
    status = tree_runner_->tick();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "BT tick threw exception: %s", e.what());
    onTreeCompleted(BT::NodeStatus::FAILURE);
    return;
  }

  if (status != BT::NodeStatus::RUNNING)
  {
    onTreeCompleted(status);
  }
}

void BtExecutive::onTreeCompleted(BT::NodeStatus status)
{
  if (completed_)
  {
    return;
  }
  completed_ = true;

  if (tick_timer_)
  {
    tick_timer_->cancel();
  }

  try
  {
    tree_runner_->haltTree();
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(logger_, "haltTree threw: %s", e.what());
  }

  RCLCPP_INFO(logger_, "BT completed with status: %s", BT::toStr(status, false).c_str());

  completion_callback_(status);
}
}  // namespace rlc_executive
