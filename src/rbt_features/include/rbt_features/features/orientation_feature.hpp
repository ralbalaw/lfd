#pragma once

#include <string>

#include "rbt_features/queries/orientation_query.hpp"
#include "rbt_types/feature.hpp"
#include "rbt_features/types.hpp"

namespace rbt_features
{

class OrientationFeature final : public rbt_types::Feature
{
public:
  struct GaussianOptions
  {
    double sigma{ 0.35 };
  };
  struct AxisOptions
  {
    Eigen::Vector3d local{ 0.0, 0.0, 1.0 };
    Eigen::Vector3d world_reference{ 0.0, 0.0, 1.0 };
  };
  struct ValueOptions
  {
    GaussianOptions gaussian;
    AxisOptions axis;
  };
  struct Options
  {
    std::string name;
    ValueOptions value;
  };

  OrientationFeature(OrientationQueryPtr query, Options opt);

  std::string name() const override
  { return opt_.name; }

  double evaluate(const JointPathView& path) const override;

  double evaluateAt(const JointVecView& q) const override;

private:
  double computeValue(const OrientationQuat& orientation) const;

  double computeTilt(const OrientationQuat& orientation) const;

private:
  OrientationQueryPtr query_;
  Options opt_;
};

}  // namespace rbt_features
