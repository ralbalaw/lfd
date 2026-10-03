#include "rbt_features/queries/tesseract_distance_query.hpp"

#include <algorithm>
#include <utility>

#include <tesseract_collision/core/types.h>

namespace rbt_features
{
namespace
{

bool contains(const std::vector<std::string>& names, const std::string& name)
{ return std::find(names.begin(), names.end(), name) != names.end(); }

}  // namespace

TesseractDistanceQuery::TesseractDistanceQuery(const EnvironmentConstPtr& env,
                                               std::string group_name,
                                               std::string source_object,
                                               std::string target_object, Options opt)
  : group_name_(std::move(group_name))
  , source_object_(std::move(source_object))
  , target_object_(std::move(target_object))
  , opt_(std::move(opt))
{
  env_ = env->clone();
  joint_names_ = env_->getGroupJointNames(group_name_);
  manager_ = env_->getDiscreteContactManager();

  configureActiveObjects();
  configureContactManager();
}

void TesseractDistanceQuery::configureActiveObjects()
{
  const std::vector<std::string> active_link_names =
      env_->getActiveLinkNames(joint_names_);

  std::vector<std::string> active_objects;
  active_objects.reserve(2);

  if (contains(active_link_names, source_object_))
  {
    active_objects.push_back(source_object_);
  }

  if (contains(active_link_names, target_object_) && target_object_ != source_object_)
  {
    active_objects.push_back(target_object_);
  }

  if (active_objects.empty())
  {
    active_objects.push_back(source_object_);
  }

  manager_->setActiveCollisionObjects(active_objects);
}

void TesseractDistanceQuery::configureContactManager()
{
  tesseract_collision::ContactManagerConfig config;

  for (const auto& object_name : manager_->getCollisionObjects())
  {
    const bool enabled = (object_name == source_object_ || object_name == target_object_);
    config.modify_object_enabled[object_name] = enabled;
  }

  config.pair_margin_data.setCollisionMargin(source_object_, target_object_,
                                             opt_.max_distance);

  config.pair_margin_override_type =
      tesseract_common::CollisionMarginPairOverrideType::MODIFY;

  manager_->applyContactManagerConfig(config);
}

double TesseractDistanceQuery::computeDistance(const JointVecView& q) const
{
  env_->setState(joint_names_, q);

  manager_->setCollisionObjectsTransform(source_object_,
                                         env_->getLinkTransform(source_object_));
  manager_->setCollisionObjectsTransform(target_object_,
                                         env_->getLinkTransform(target_object_));

  tesseract_collision::ContactRequest request;
  request.type = tesseract_collision::ContactTestType::CLOSEST;
  request.calculate_distance = true;
  request.calculate_penetration = false;

  tesseract_collision::ContactResultMap results;
  manager_->contactTest(results, request);

  return extractDistance(results);
}

double TesseractDistanceQuery::extractDistance(
    const tesseract_collision::ContactResultMap& results) const
{
  tesseract_collision::ContactResultVector flat_results;
  results.flattenCopyResults(flat_results);

  double distance = opt_.max_distance;

  for (const auto& result : flat_results)
  {
    const auto& link_names = result.link_names;
    const bool forward =
        (link_names[0] == source_object_ && link_names[1] == target_object_);
    const bool reverse =
        (link_names[0] == target_object_ && link_names[1] == source_object_);

    if (forward || reverse)
    {
      distance = result.distance;
      break;
    }
  }

  return distance;
}

}  // namespace rbt_features