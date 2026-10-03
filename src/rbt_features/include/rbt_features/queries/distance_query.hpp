#pragma once

#include <Eigen/Core>
#include <memory>

#include "rbt_features/types.hpp"

namespace rbt_features
{

class DistanceQuery
{
public:
  using Ptr = std::shared_ptr<DistanceQuery>;
  using ConstPtr = std::shared_ptr<const DistanceQuery>;

  virtual ~DistanceQuery() = default;

  virtual double computeDistance(const JointVecView& joint_vec) const = 0;
};

using DistanceQueryPtr = DistanceQuery::Ptr;
using DistanceQueryConstPtr = DistanceQuery::ConstPtr;

}  // namespace rbt_features