#include "rbt_features/queries/tesseract_orientation_query.hpp"

#include <Eigen/Geometry>
namespace rbt_features
{

TesseractOrientationQuery::TesseractOrientationQuery(EnvironmentConstPtr env,
                                                     std::string group_name,
                                                     std::string link_name)
  : group_name_(std::move(group_name)), link_name_(std::move(link_name))
{
  env_ = env->clone();
  joint_names_ = env_->getGroupJointNames(group_name_);
}

OrientationQuat TesseractOrientationQuery::computeOrientation(const JointVecView& q) const
{
  env_->setState(joint_names_, q);

  const Eigen::Isometry3d transform = env_->getLinkTransform(link_name_);

  OrientationQuat quat(transform.linear());
  quat.normalize();
  return quat;
}

}  // namespace rbt_features