#pragma once

#include "rlc_frontend/collect_demonstration_parameters.hpp"
#include "rlc_frontend/frontend/frontend_config.hpp"

namespace rlc_frontend
{
namespace config
{

/**
 * @brief Converts generated ROS parameters into frontend runtime config.
 * @param[in] params Generated parameter snapshot read by `ParamListener`.
 * @return Demonstration collection runtime configuration.
 */
CollectDemonstrationConfig
loadCollectDemonstrationConfigFromParameters(const rlc_frontend::Params& params);

}  // namespace config
}  // namespace rlc_frontend
