#pragma once

#include "rbt_types/feature.hpp"
#include "rbt_types/joint_endpoints.hpp"
#include "rbt_types/joint_path.hpp"
#include "rbt_types/math.hpp"
#include "rbt_types/objective_term.hpp"

namespace rbt_irl
{

/// @brief Joint path used by the IRL package.
using JointPath = rbt_types::JointPath;

/// @brief Collection of joint paths.
using JointPaths = rbt_types::JointPaths;

/// @brief Start and goal joint positions.
using JointEndpoints = rbt_types::JointEndpoints;

/// @brief Collection of joint endpoints.
using JointEndpointsSet = rbt_types::JointEndpointsSet;

/// @brief Collection of feature implementations.
using Features = rbt_types::Features;

/// @brief IRL parameter vector.
using WeightVec = rbt_types::WeightVec;

/// @brief Feature-value vector.
using FeatureVec = rbt_types::FeatureVec;

/// @brief Collection of weighted reward terms.
using RewardTerms = rbt_types::RewardTerms;

}  // namespace rbt_irl
