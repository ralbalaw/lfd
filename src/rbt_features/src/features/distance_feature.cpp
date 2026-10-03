#include "rbt_features/features/distance_feature.hpp"

#include <utility>

#include <rbt_types/math.hpp>

#include "rbt_features/math.hpp"

namespace rbt_features
{

DistanceFeature::DistanceFeature(DistanceQueryPtr query, Options opt)
  : query_(std::move(query)), opt_(std::move(opt))
{
}

double DistanceFeature::evaluate(const JointPathView& path) const
{
  double total = 0.0;
  const Eigen::Index rows = path.rows();
  const Eigen::Index cols = path.cols();
  const double* data = path.data();

  for (Eigen::Index k = 0; k < rows; ++k)
  {
    Eigen::Map<const JointVec> q(data + k * cols, cols);
    total += evaluateAt(q);
  }

  return total;
}

double DistanceFeature::evaluateAt(const JointVecView& q) const
{
  const double distance = query_->computeDistance(q);
  return computeValue(distance);
}

double DistanceFeature::computeValue(double distance) const
{
  const double clearance = std::max(distance, 0.0);

  const double slope = opt_.value.sigmoid.slope;
  const double midpoint = opt_.value.sigmoid.midpoint;

  return math::normalizedSigmoid(clearance, slope, midpoint);
}
}  // namespace rbt_features
