#include "rlc_executive/config/planning_config_registry.hpp"

#include <stdexcept>
#include <utility>

namespace rlc_executive
{

PlanningConfigRegistry::PlanningConfigRegistry(std::string default_config,
                                               PlanningConfigMap configs)
  : default_config_(std::move(default_config)), configs_(std::move(configs))
{
  validateDefaultConfig();
}

const PlanningConfig* PlanningConfigRegistry::find(const std::string& name) const noexcept
{
  const auto it = configs_.find(name);
  if (it == configs_.end())
  {
    return nullptr;
  }
  return &it->second;
}

const PlanningConfig& PlanningConfigRegistry::defaultConfig() const noexcept
{
  return configs_.find(default_config_)->second;
}

const std::string& PlanningConfigRegistry::defaultConfigName() const noexcept
{
  return default_config_;
}

bool PlanningConfigRegistry::contains(const std::string& name) const noexcept
{
  return find(name) != nullptr;
}

void PlanningConfigRegistry::validateDefaultConfig() const
{
  if (configs_.empty())
  {
    throw std::invalid_argument("PlanningConfigRegistry: configs must not be empty");
  }
  if (default_config_.empty())
  {
    throw std::invalid_argument(
        "PlanningConfigRegistry: default_config must not be empty");
  }
  if (!contains(default_config_))
  {
    throw std::invalid_argument("PlanningConfigRegistry: default_config '" +
                                default_config_ + "' was not loaded");
  }
}

}  // namespace rlc_executive
