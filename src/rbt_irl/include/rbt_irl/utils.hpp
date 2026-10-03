#pragma once

#include "rbt_irl/types.hpp"

namespace rbt_irl
{
namespace utils
{

/**
 * @brief Extracts the endpoints of each joint path.
 * @param[in] paths Joint paths, length = num_paths.
 * @return Joint endpoints, length = `paths.size()`.
 */
inline JointEndpointsSet extractJointEndpoints(const JointPaths& paths)
{
  JointEndpointsSet endpoints;
  endpoints.reserve(paths.size());

  for (const JointPath& path : paths)
  {
    const auto& positions = path.positions();
    endpoints.push_back({ positions.row(0).transpose(),
                          positions.row(positions.rows() - 1).transpose() });
  }

  return endpoints;
}

}  // namespace utils
}  // namespace rbt_irl
