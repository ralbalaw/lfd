#pragma once

#include <string>

#include <rlc_executive/bt_executive_parameters.hpp>

#include "rlc_executive/config/executive_config.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"

namespace rlc_executive
{
namespace config
{
/**
 * @brief Converts generated ROS parameters into the executive runtime config.
 * @param[in] params Generated parameter snapshot read by `ParamListener`.
 * @return Executive runtime configuration.
 * @throws std::invalid_argument If a generated enum token or value is invalid.
 */
ExecutiveConfig loadExecutiveConfigFromParameters(const rlc_executive::Params& params);

/**
 * @brief Loads planning configs from a YAML file.
 * @param[in] yaml_path Filesystem path to the planning configs YAML.
 * @param[in] default_config Default config name; may be empty.
 * @return Parsed planning-config registry.
 * @throws std::runtime_error If the file cannot be loaded or lacks a `configs` map.
 * @throws std::invalid_argument If a config entry is invalid.
 * @throws YAML::Exception If a YAML value cannot be converted.
 */
PlanningConfigRegistry
loadPlanningConfigRegistryFromYaml(const std::string& yaml_path,
                                   const std::string& default_config);
}  // namespace config
}  // namespace rlc_executive
