#pragma once

#include <Eigen/Core>
#include <memory>

#include "rbt_features/types.hpp"
namespace rbt_features
{

class OrientationQuery
{
public:  
  using Ptr = std::shared_ptr<OrientationQuery>;
  using ConstPtr = std::shared_ptr<const OrientationQuery>;

  virtual ~OrientationQuery() = default;

  virtual OrientationQuat computeOrientation(const JointVecView& joint_vec) const = 0;
};

using OrientationQueryPtr = OrientationQuery::Ptr;
using OrientationQueryConstPtr = OrientationQuery::ConstPtr;

}  // namespace rbt_features