#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "rlc_executive/config/config_loader.hpp"

namespace rlc_executive
{
namespace config
{
namespace
{

bool hasValue(const YAML::Node& node)
{
  return node && !node.IsNull();
}

void requireMap(const YAML::Node& node)
{
  if (!node || node.IsNull() || !node.IsMap())
  {
    throw std::invalid_argument("expected YAML map");
  }
}

template <typename T>
void readOptional(const YAML::Node& node, const std::string& key, T* config)
{
  const YAML::Node child = node[key];
  if (hasValue(child))
  {
    *config = child.as<T>();
  }
}

std::string requireString(const YAML::Node& node, const std::string& key,
                          const std::string& context)
{
  const YAML::Node child = node[key];
  if (!hasValue(child))
  {
    throw std::invalid_argument("missing YAML key '" + context + "." + key + "'");
  }
  return child.as<std::string>();
}

void validatePlanningConfig(const PlanningConfig& config, const std::string& context)
{
  if (config.pipeline_id.empty())
  {
    throw std::invalid_argument(context + ".pipeline_id must not be empty");
  }
  if (config.planner_id.empty())
  {
    throw std::invalid_argument(context + ".planner_id must not be empty");
  }
  if (config.allowed_planning_time_sec <= 0.0)
  {
    throw std::invalid_argument(context + ".allowed_planning_time_sec must be > 0");
  }
  if (config.num_planning_attempts < 1)
  {
    throw std::invalid_argument(context + ".num_planning_attempts must be >= 1");
  }
  if (config.max_velocity_scaling_factor <= 0.0 ||
      config.max_velocity_scaling_factor > 1.0)
  {
    throw std::invalid_argument(context +
                                ".max_velocity_scaling_factor must be in (0, 1]");
  }
  if (config.max_acceleration_scaling_factor <= 0.0 ||
      config.max_acceleration_scaling_factor > 1.0)
  {
    throw std::invalid_argument(context +
                                ".max_acceleration_scaling_factor must be in (0, 1]");
  }
}

PlanningConfig parsePlanningConfig(const YAML::Node& node, const std::string& name)
{
  requireMap(node);

  const std::string context = "configs." + name;
  if (hasValue(node["name"]))
  {
    throw std::invalid_argument(context + ".name is not supported; use the map key");
  }

  PlanningConfig config;
  config.pipeline_id = requireString(node, "pipeline_id", context);
  config.planner_id = requireString(node, "planner_id", context);
  readOptional(node, "allowed_planning_time_sec", &config.allowed_planning_time_sec);
  readOptional(node, "num_planning_attempts", &config.num_planning_attempts);
  readOptional(node, "max_velocity_scaling_factor", &config.max_velocity_scaling_factor);
  readOptional(node, "max_acceleration_scaling_factor",
               &config.max_acceleration_scaling_factor);

  validatePlanningConfig(config, context);
  return config;
}

PlanningConfigMap parsePlanningConfigs(const YAML::Node& root)
{
  requireMap(root);

  const YAML::Node configs = root["configs"];
  if (!hasValue(configs) || !configs.IsMap())
  {
    throw std::runtime_error("Planning configs YAML must contain a map at key 'configs'");
  }

  PlanningConfigMap parsed_configs;
  for (auto it = configs.begin(); it != configs.end(); ++it)
  {
    const std::string config_name = it->first.as<std::string>();
    if (config_name.empty())
    {
      throw std::invalid_argument("planning config name must not be empty");
    }
    if (parsed_configs.contains(config_name))
    {
      throw std::invalid_argument("duplicate planning config: " + config_name);
    }

    parsed_configs.emplace(config_name, parsePlanningConfig(it->second, config_name));
  }

  return parsed_configs;
}

YAML::Node loadYamlFile(const std::string& yaml_path, const std::string& label)
{
  try
  {
    return YAML::LoadFile(yaml_path);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("Failed to load " + label + " YAML: '" + yaml_path +
                             "'. Details: " + e.what());
  }
}

}  // namespace

PlanningConfigRegistry
loadPlanningConfigRegistryFromYaml(const std::string& yaml_path,
                                   const std::string& default_config)
{
  const YAML::Node root = loadYamlFile(yaml_path, "planning configs");
  return PlanningConfigRegistry(default_config, parsePlanningConfigs(root));
}

}  // namespace config
}  // namespace rlc_executive
