#pragma once

#include <string>
#include <variant>

#include <Eigen/Core>

#include "rbt_types/math.hpp"
#include "rbt_types/geometry.hpp"
namespace rbt_features
{

using JointVec = rbt_types::JointVec;
using JointRowVec = rbt_types::JointRowVec;
using JointVecView = rbt_types::JointVecView;
using JointRowVecView = rbt_types::JointRowVecView;

using JointPathMat = rbt_types::JointPathMat;
using JointPathView = rbt_types::JointPathView;

using OrientationQuat = rbt_types::OrientationQuat;

/**
 * @brief Point rigidly attached to a named frame.
 *
 * @details
 * - `frame_name`: Frame identifier used by the distance-query backend.
 * - `offset_in_frame`: Point coordinates [m], size = 3, expressed in `frame_name`.
 */
struct FramePoint
{
  std::string frame_name;
  Eigen::Vector3d offset_in_frame{ Eigen::Vector3d::Zero() };
};

/**
 * @brief Reference to a named collision geometry.
 *
 * @details
 * - `name`: Geometry identifier understood by the distance-query backend.
 */
struct GeometryRef
{
  std::string name;
};

/// @brief Distance-query entity, either a frame-attached point or a geometry reference.
using Entity = std::variant<FramePoint, GeometryRef>;

}  // namespace rbt_features
