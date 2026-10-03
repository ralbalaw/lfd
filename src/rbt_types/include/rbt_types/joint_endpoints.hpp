#pragma once

#include <vector>

#include "rbt_types/math.hpp"

namespace rbt_types
{

/// @brief Start and goal joint positions [rad], each size = dof.
struct JointEndpoints
{
  JointVec start;
  JointVec goal;
};

/// @brief A collection of joint-space start/goal pairs.
using JointEndpointsSet = std::vector<JointEndpoints>;

}  // namespace rbt_types
