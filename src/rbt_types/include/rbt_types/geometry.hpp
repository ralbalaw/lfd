#pragma once

#include <Eigen/Geometry>

namespace rbt_types
{
/// @brief Double-precision quaternion used for 3D rotations.
using Quat = Eigen::Quaterniond;

/// @brief Semantic alias for a quaternion representing a 3D orientation.
using OrientationQuat = Quat;

}  // namespace rbt_types
