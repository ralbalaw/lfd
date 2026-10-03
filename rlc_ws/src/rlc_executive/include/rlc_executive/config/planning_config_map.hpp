#pragma once

#include <map>
#include <string>

#include "rlc_executive/config/planning_config.hpp"

namespace rlc_executive
{

/// @brief Named planning configs keyed by their YAML config names.
using PlanningConfigMap = std::map<std::string, PlanningConfig>;

}  // namespace rlc_executive
