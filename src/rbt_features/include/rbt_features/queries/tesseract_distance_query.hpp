#pragma once

#include "rbt_features/queries/distance_query.hpp"

#include <memory>
#include <string>
#include <vector>

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_environment/environment.h>

#include "rbt_features/types.hpp"

namespace rbt_features
{

/**
 * @brief Computes the closest distance between two entities in a Tesseract environment.
 *
 * @details
 * The query clones the provided environment during construction and evaluates distances
 * for joint configurations expressed in the selected group's joint order.
 */
class TesseractDistanceQuery final : public DistanceQuery
{
public:
  using EnvironmentConstPtr = tesseract_environment::Environment::ConstPtr;

  struct Options
  {
    double max_distance{
      1.0
    };  ///< Maximum distance to report; larger distances are clamped to this value.
  };

  /**
   * @brief Constructs the query.
   * @param[in] env Tesseract environment to query. The environment is cloned internally.
   * @param[in] group_name Name of the joint group to use for distance evaluations.
   * @param[in] source_object Name of the first entity (link or collision object) to evaluate.
   * @param[in] target_object Name of the second entity (link or collision object) to evaluate.
   * @param[in] opt Configuration options for the query.
   */
  TesseractDistanceQuery(const EnvironmentConstPtr& env, std::string group_name,
                         std::string source_object, std::string target_object,
                         Options opt);

  void configureContactManager();
  void configureActiveObjects();

  /**
   * @brief Evaluates the closest distance for a joint configuration.
   * @param[in] q Joint values [rad or m], size = number of joints in `group_name`,
   *   row-vector layout, in the group's joint order.
   * @return Closest signed distance [m] between the configured source and target entities.
   */
  double computeDistance(const JointVecView& q) const override;

private:
  double extractDistance(const tesseract_collision::ContactResultMap& results) const;

  std::string group_name_;
  std::string source_object_;
  std::string target_object_;

  mutable tesseract_environment::Environment::UPtr env_;
  mutable std::unique_ptr<tesseract_collision::DiscreteContactManager> manager_;

  std::vector<std::string> joint_names_;

  Options opt_;
};

}  // namespace rbt_features
