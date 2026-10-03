#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rlc_scene_bridge/moveit_tesseract_bridge.hpp>
#include <rlc_scene_bridge/scene_bridge_options.hpp>
#include <rlc_utils/tesseract_utils.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

namespace
{

template <typename T>
T declareOrGetParameter(rclcpp::Node& node, const std::string& name,
                        const T& default_value)
{
  if (!node.has_parameter(name))
  {
    try
    {
      return node.declare_parameter<T>(name, default_value);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
      // Parameter may be auto-declared from overrides; fall through to get().
    }
  }

  T value = default_value;
  (void)node.get_parameter(name, value);
  return value;
}

rlc_scene_bridge::MoveItTesseractBridgeOptions loadBridgeOptions(rclcpp::Node& node,
                                                                 const std::string& pfx)
{
  using rlc_scene_bridge::MoveItTesseractBridgeOptions;

  MoveItTesseractBridgeOptions opt{};

  opt.monitor_namespace = declareOrGetParameter<std::string>(
      node, pfx + "monitor_namespace", opt.monitor_namespace);
  opt.env_name = declareOrGetParameter<std::string>(node, pfx + "env_name", opt.env_name);
  opt.scene_topic =
      declareOrGetParameter<std::string>(node, pfx + "scene_topic", opt.scene_topic);
  opt.get_scene_srv =
      declareOrGetParameter<std::string>(node, pfx + "get_scene_srv", opt.get_scene_srv);

  const int scene_components = declareOrGetParameter<int>(
      node, pfx + "scene_components", static_cast<int>(opt.scene_components));
  if (scene_components >= 0)
  {
    opt.scene_components = static_cast<uint32_t>(scene_components);
  }

  const int srv_wait_ms = declareOrGetParameter<int>(
      node, pfx + "srv_wait_ms", static_cast<int>(opt.srv_wait.count()));
  if (srv_wait_ms >= 0)
  {
    opt.srv_wait = std::chrono::milliseconds{ srv_wait_ms };
  }

  const int scene_qos_depth = declareOrGetParameter<int>(
      node, pfx + "scene_qos_depth", static_cast<int>(opt.scene_qos_depth));
  if (scene_qos_depth > 0)
  {
    opt.scene_qos_depth = static_cast<std::size_t>(scene_qos_depth);
  }

  opt.env_sync.allow_replace = declareOrGetParameter<bool>(
      node, pfx + "env_sync.allow_replace", opt.env_sync.allow_replace);

  opt.env_sync.naming_policy.world_link_prefix =
      declareOrGetParameter<std::string>(node,
                                         pfx + "env_sync.naming_policy.world_link_prefix",
                                         opt.env_sync.naming_policy.world_link_prefix);
  opt.env_sync.naming_policy.attached_link_prefix = declareOrGetParameter<std::string>(
      node, pfx + "env_sync.naming_policy.attached_link_prefix",
      opt.env_sync.naming_policy.attached_link_prefix);

  return opt;
}

}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("rlc_scene_bridge", node_options);
  auto logger = node->get_logger();

  const std::string pfx = "scene_bridge.";
  auto bridge_opt = loadBridgeOptions(*node, pfx);

  const int monitor_wait_ms =
      declareOrGetParameter<int>(*node, pfx + "monitor_wait_ms", 3000);
  const int resync_period_ms =
      declareOrGetParameter<int>(*node, pfx + "resync_period_ms", 1000);

  const auto monitor_wait = std::chrono::milliseconds{ std::max(0, monitor_wait_ms) };

  auto mon_opt = rlc_utils::tesseract_utils::makeMonitorInterface(
      node, logger, bridge_opt.monitor_namespace, monitor_wait, bridge_opt.env_name);
  if (!mon_opt)
  {
    RCLCPP_FATAL(logger,
                 "Failed to create Tesseract monitor interface for namespace '%s'",
                 bridge_opt.monitor_namespace.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::shared_ptr<rlc_scene_bridge::MoveItTesseractBridge> bridge;
  auto mon = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitorInterface>(
      std::move(*mon_opt));
  try
  {
    rlc_scene_bridge::MoveItTesseractBridge::MonitorInterfaceConstPtr mon_const = mon;
    bridge = std::make_shared<rlc_scene_bridge::MoveItTesseractBridge>(node, mon_const,
                                                                       bridge_opt);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(logger, "Failed to construct MoveItTesseractBridge: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  const auto resync_period = std::chrono::milliseconds{ std::max(50, resync_period_ms) };
  auto resync_timer = node->create_wall_timer(resync_period, [bridge]() {
    if (bridge && !bridge->isSynchronized())
    {
      bridge->requestSync();
    }
  });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  resync_timer.reset();
  bridge.reset();
  mon.reset();

  rclcpp::shutdown();
  return 0;
}
