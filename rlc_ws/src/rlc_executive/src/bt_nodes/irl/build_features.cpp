#include "rlc_executive/bt_nodes/irl/build_features.hpp"

#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rbt_features/features/distance_feature.hpp"
#include "rbt_features/features/orientation_feature.hpp"
#include "rbt_features/queries/tesseract_distance_query.hpp"
#include "rbt_features/queries/tesseract_orientation_query.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{
namespace
{

bool hasConfiguredValue(const BT::Expected<std::string>& value)
{
  return value && !value.value().empty();
}

std::shared_ptr<tesseract_environment::Environment>
snapshotEnvironment(const BuildFeatures& node,
                    BuildFeatures::EnvMonitorInterface* env_monitor_interface,
                    const std::string& monitor_namespace)
{
  auto env_uptr = env_monitor_interface->getEnvironment(monitor_namespace);
  if (!env_uptr)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: failed to snapshot Tesseract environment from namespace '",
                           monitor_namespace, "'");
  }

  return std::shared_ptr<tesseract_environment::Environment>{ std::move(env_uptr) };
}

}  // namespace

BuildFeatures::BuildFeatures(const std::string& name, const BT::NodeConfig& config,
                             rclcpp::Logger logger,
                             EnvMonitorInterface* env_monitor_interface,
                             const std::string& monitor_namespace)
  : BT::SyncActionNode(name, config)
  , logger_(logger.get_child(bt_utils::sanitizeLoggerName(fullPath())))
  , env_monitor_interface_(bt_utils::requireNonNull(
        env_monitor_interface, *this, "EnvironmentMonitorInterface pointer"))
  , monitor_namespace_(monitor_namespace)
{
  if (monitor_namespace_.empty())
  {
    throw BT::LogicError(fullPath(), "]: monitor_namespace is empty");
  }
}

BT::NodeStatus BuildFeatures::tick()
{
  try
  {
    const std::string group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::kGroupName);
    if (group_name.empty())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::kGroupName, "' must not be empty");
    }

    const auto distance_name_in = getInput<std::string>(PortKeys::kDistanceName);
    const auto distance_source_in =
        getInput<std::string>(PortKeys::kDistanceSourceObject);
    const auto distance_target_in =
        getInput<std::string>(PortKeys::kDistanceTargetObject);

    const auto orientation_name_in = getInput<std::string>(PortKeys::kOrientationName);
    const auto orientation_link_in =
        getInput<std::string>(PortKeys::kOrientationLinkName);

    const bool build_distance = hasConfiguredValue(distance_name_in) ||
                                hasConfiguredValue(distance_source_in) ||
                                hasConfiguredValue(distance_target_in);
    const bool build_orientation = hasConfiguredValue(orientation_name_in) ||
                                   hasConfiguredValue(orientation_link_in);

    if (!build_distance && !build_orientation)
    {
      throw BT::RuntimeError(
          registrationName(), "[", fullPath(),
          "]: no feature inputs configured; set distance_* and/or orientation_* ports");
    }

    const auto env =
        snapshotEnvironment(*this, env_monitor_interface_, monitor_namespace_);

    Features features;
    features.reserve(static_cast<std::size_t>(build_distance) +
                     static_cast<std::size_t>(build_orientation));

    if (build_distance)
    {
      const std::string feature_name =
          bt_utils::requireInput<std::string>(*this, PortKeys::kDistanceName);
      const std::string source_object =
          bt_utils::requireInput<std::string>(*this, PortKeys::kDistanceSourceObject);
      const std::string target_object =
          bt_utils::requireInput<std::string>(*this, PortKeys::kDistanceTargetObject);

      if (feature_name.empty() || source_object.empty() || target_object.empty())
      {
        throw BT::RuntimeError(registrationName(), "[", fullPath(),
                               "]: distance feature ports must not be empty");
      }

      rbt_features::TesseractDistanceQuery::Options query_opt;
      query_opt.max_distance =
          bt_utils::requireInput<double>(*this, PortKeys::kDistanceMaxDistance);
      if (!(query_opt.max_distance > 0.0))
      {
        throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                               PortKeys::kDistanceMaxDistance, "' must be > 0");
      }

      rbt_features::DistanceFeature::Options feature_opt;
      feature_opt.name = feature_name;
      feature_opt.value.sigmoid.slope =
          bt_utils::requireInput<double>(*this, PortKeys::kDistanceSigmoidSlope);
      feature_opt.value.sigmoid.midpoint =
          bt_utils::requireInput<double>(*this, PortKeys::kDistanceSigmoidMidpoint);

      auto query = std::make_shared<rbt_features::TesseractDistanceQuery>(
          env, group_name, source_object, target_object, query_opt);
      features.push_back(
          std::make_shared<rbt_features::DistanceFeature>(std::move(query), feature_opt));
    }

    if (build_orientation)
    {
      const std::string feature_name =
          bt_utils::requireInput<std::string>(*this, PortKeys::kOrientationName);
      const std::string link_name =
          bt_utils::requireInput<std::string>(*this, PortKeys::kOrientationLinkName);

      if (feature_name.empty() || link_name.empty())
      {
        throw BT::RuntimeError(registrationName(), "[", fullPath(),
                               "]: orientation feature ports must not be empty");
      }

      rbt_features::OrientationFeature::Options feature_opt;
      feature_opt.name = feature_name;
      feature_opt.value.gaussian.sigma =
          bt_utils::requireInput<double>(*this, PortKeys::kOrientationSigma);
      if (!(feature_opt.value.gaussian.sigma > 0.0))
      {
        throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                               PortKeys::kOrientationSigma, "' must be > 0");
      }

      auto query = std::make_shared<rbt_features::TesseractOrientationQuery>(
          env, group_name, link_name);
      features.push_back(std::make_shared<rbt_features::OrientationFeature>(
          std::move(query), feature_opt));
    }

    bt_utils::setOutput(*this, PortKeys::kFeatures, features);
    bt_utils::setOutput(*this, PortKeys::kError, std::string());

    RCLCPP_INFO(logger_, "built %zu feature(s) for group='%s'", features.size(),
                group_name.c_str());

    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus BuildFeatures::failWithError(const std::string& msg)
{
  bt_utils::setOutput(*this, PortKeys::kFeatures, Features{});
  bt_utils::setOutput(*this, PortKeys::kError, msg);
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace rlc_executive
