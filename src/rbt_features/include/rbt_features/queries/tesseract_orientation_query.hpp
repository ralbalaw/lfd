#pragma once

#include "rbt_features/queries/orientation_query.hpp"
#include "rbt_features/types.hpp"

#include <string>
#include <vector>

#include <tesseract_environment/environment.h>

namespace rbt_features
{

class TesseractOrientationQuery final : public OrientationQuery
{
public:
  using EnvironmentConstPtr = tesseract_environment::Environment::ConstPtr;

  TesseractOrientationQuery(EnvironmentConstPtr env, std::string group_name,
                            std::string link_name);

  OrientationQuat computeOrientation(const JointVecView& q) const override;

private:
  mutable tesseract_environment::Environment::UPtr env_;

  std::string group_name_;
  std::string link_name_;
  std::vector<std::string> joint_names_;
};

}  // namespace rbt_features