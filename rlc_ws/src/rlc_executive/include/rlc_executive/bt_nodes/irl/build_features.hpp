#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/logger.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rbt_types/feature.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

/**
 * @brief Builds a feature set for IRL and TrajOpt reward construction.
 *
 * @details
 * This BT node snapshots the monitored Tesseract environment once per tick and builds
 * a `rbt_types::Features` vector from the configured feature inputs.
 *
 * Supported feature forms:
 * - Distance feature: provide `distance_name`, `distance_source_object`, and
 *   `distance_target_object`.
 * - Orientation feature: provide `orientation_name` and `orientation_link_name`.
 *
 * At least one feature form must be configured.
 */
class BuildFeatures final : public BT::SyncActionNode
{
public:
  using Features = rbt_types::Features;
  using EnvMonitorInterface = tesseract_monitoring::ROSEnvironmentMonitorInterface;

  /**
   * @brief Constructs the feature-builder BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   * @param[in] logger Logger used for diagnostics.
   * @param[in] env_monitor_interface Retained Tesseract monitor interface; must not be
   * nullptr and must outlive this BT node.
   * @param[in] monitor_namespace Tesseract monitor namespace used for environment
   * snapshots; must not be empty.
   */
  BuildFeatures(const std::string& name, const BT::NodeConfig& config,
                rclcpp::Logger logger, EnvMonitorInterface* env_monitor_interface,
                const std::string& monitor_namespace);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing the manipulator group, optional distance
   * and orientation feature configuration, the built feature set, and an error output.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::kGroupName),

      BT::InputPort<std::string>(PortKeys::kDistanceName),
      BT::InputPort<std::string>(PortKeys::kDistanceSourceObject),
      BT::InputPort<std::string>(PortKeys::kDistanceTargetObject),
      BT::InputPort<double>(PortKeys::kDistanceMaxDistance, 1.0, ""),
      BT::InputPort<double>(PortKeys::kDistanceSigmoidSlope, 1.0, ""),
      BT::InputPort<double>(PortKeys::kDistanceSigmoidMidpoint, 0.0, ""),

      BT::InputPort<std::string>(PortKeys::kOrientationName),
      BT::InputPort<std::string>(PortKeys::kOrientationLinkName),
      BT::InputPort<double>(PortKeys::kOrientationSigma, 0.35, ""),

      BT::OutputPort<Features>(PortKeys::kFeatures),
      BT::OutputPort<std::string>(PortKeys::kError),
    };
  }

  /**
   * @brief Names of BT ports used by BuildFeatures.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string kGroupName = "group_name";

    static inline const std::string kDistanceName = "distance_name";
    static inline const std::string kDistanceSourceObject = "distance_source_object";
    static inline const std::string kDistanceTargetObject = "distance_target_object";
    static inline const std::string kDistanceMaxDistance = "distance_max_distance";
    static inline const std::string kDistanceSigmoidSlope = "distance_sigmoid_slope";
    static inline const std::string kDistanceSigmoidMidpoint =
        "distance_sigmoid_midpoint";

    static inline const std::string kOrientationName = "orientation_name";
    static inline const std::string kOrientationLinkName = "orientation_link_name";
    static inline const std::string kOrientationSigma = "orientation_sigma";

    static inline const std::string kFeatures = "features";
  };

  /**
   * @brief Builds the configured feature set and writes it to the blackboard.
   * @return `SUCCESS` if at least one feature is built, otherwise `FAILURE`.
   */
  BT::NodeStatus tick() override;

private:
  BT::NodeStatus failWithError(const std::string& msg);

  rclcpp::Logger logger_;
  EnvMonitorInterface* env_monitor_interface_;
  std::string monitor_namespace_;
};

}  // namespace rlc_executive
