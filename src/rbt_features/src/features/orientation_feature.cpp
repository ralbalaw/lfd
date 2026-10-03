#include "rbt_features/features/orientation_feature.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/src/Core/Matrix.h>

#include "rbt_features/math.hpp"

namespace rbt_features
{

OrientationFeature::OrientationFeature(OrientationQueryPtr query, Options opt)
  : query_(std::move(query)), opt_(std::move(opt))
{
}

double OrientationFeature::evaluate(const JointPathView& path) const
{
  double total = 0.0;
  const Eigen::Index rows = path.rows();
  const Eigen::Index cols = path.cols();
  const double* data = path.data();

  for (Eigen::Index k = 0; k < rows; ++k)
  {
    Eigen::Map<const JointVec> q_vec(data + k * cols, cols);
    total += evaluateAt(q_vec);
  }

  return total;
}

double OrientationFeature::evaluateAt(const JointVecView& q) const
{
  const OrientationQuat orientation = query_->computeOrientation(q);
  return computeValue(orientation);
}

double OrientationFeature::computeTilt(const OrientationQuat& orientation) const
{
  const auto& opt = opt_.value;
  const auto& axis_l = opt.axis.local;
  const auto& axis_w_ref = opt.axis.world_reference;

  const Eigen::Vector3d axis_w = (orientation * axis_l);
  const double cos_tilt = axis_w.dot(axis_w_ref);
  const double cos_tilt_clamped = std::clamp(cos_tilt, -1.0, 1.0);
  return std::acos(cos_tilt_clamped);
}

double OrientationFeature::computeValue(const OrientationQuat& orientation) const
{
  const double tilt = computeTilt(orientation);
  return math::gaussian(tilt, opt_.value.gaussian.sigma);
}
}  // namespace rbt_features
