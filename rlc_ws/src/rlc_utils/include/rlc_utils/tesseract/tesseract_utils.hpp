#pragma once

#include <chrono>
#include <optional>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

namespace rlc_utils::tesseract_utils
{

/**
 * @brief Creates a Tesseract monitor interface for an explicit environment name.
 * @param[in] node ROS node used by the monitor interface; must not be null.
 * @param[in] logger Logger used for construction and connection failures.
 * @param[in] monitor_namespace Tesseract monitor service namespace; must not be
 * empty.
 * @param[in] timeout Maximum time [ms] to wait for the monitor namespace.
 * @param[in] env_name Tesseract environment name; must not be empty.
 * @return Monitor interface on success, or `std::nullopt` if validation or
 * namespace connection fails.
 */

inline std::optional<tesseract_monitoring::ROSEnvironmentMonitorInterface>
makeMonitorInterface(rclcpp::Node::SharedPtr node, const rclcpp::Logger& logger,
                     const std::string& monitor_namespace,
                     std::chrono::milliseconds timeout, const std::string& env_name)
{
  if (!node)
  {
    RCLCPP_ERROR(logger, "Tesseract monitor node is null");
    return std::nullopt;
  }

  std::optional<tesseract_monitoring::ROSEnvironmentMonitorInterface> monitor_interface{
    std::in_place, std::move(node), env_name
  };

  monitor_interface->addNamespace(monitor_namespace);

  if (!monitor_interface->waitForNamespace(monitor_namespace, timeout))
  {
    RCLCPP_ERROR(logger,
                 "Tesseract monitor not reachable under namespace '%s' after %ld ms",
                 monitor_namespace.c_str(), static_cast<long>(timeout.count()));
    return std::nullopt;
  }

  return monitor_interface;
}

inline rclcpp::Node::SharedPtr makeMonitorClientNode(rclcpp::Node& node)
{
  rclcpp::NodeOptions monitor_node_options;
  monitor_node_options.context(node.get_node_base_interface()->get_context());

  monitor_node_options.use_global_arguments(false);
  monitor_node_options.start_parameter_services(false);
  monitor_node_options.start_parameter_event_publisher(false);

  auto monitor_node = std::make_shared<rclcpp::Node>(
      std::string(node.get_name()) + "_tesseract_monitor_client", node.get_namespace(),
      monitor_node_options);

  return monitor_node;
}

}  // namespace rlc_utils::tesseract_utils
