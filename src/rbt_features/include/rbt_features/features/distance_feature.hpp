#pragma once

#include <string>

#include "rbt_features/queries/distance_query.hpp"
#include "rbt_types/feature.hpp"
#include "rbt_features/types.hpp"

namespace rbt_features
{

/**
 * @brief Evaluates the cumulative signed distance between two entities along a trajectory.
 *
 * @details
 * The feature queries the configured DistanceQuery at every waypoint and returns
 * the sum of signed distances [m] over the full trajectory.
 */
class DistanceFeature final : public rbt_types::Feature
{
public:
  struct SigmoidOptions
  {
    double slope{ 1.0 };
    double midpoint{ 0.0 };
  };
  struct ValueOptions
  {
    SigmoidOptions sigmoid;
  };
  struct Options
  {
    ValueOptions value;
    std::string name;
  };

  /**
   * @brief Constructs a distance feature backed by a distance-query provider.
   * @param[in] query Shared distance-query; must not be `nullptr`.
   * @param[in] opt Configuration options for the feature.
   * @throws std::invalid_argument If `query == nullptr`.
   */
  DistanceFeature(DistanceQueryPtr query, Options opt);

  /// @brief Returns the configured feature name.
  std::string name() const override
  { return opt_.name; }

  /**
   * @brief Evaluates the sum of signed distances over all trajectory waypoints.
   * @param[in] path Joint-space trajectory with `shape = (num_waypoints, dof)`.
   * @return Sum of signed distances [m] over the full trajectory.
   * @throws std::invalid_argument If `path` has zero waypoints or zero joint dimension.
   */
  double evaluate(const JointPathView& path) const override;

  /**
   * @brief Evaluates the signed distance at one trajectory waypoint.
   * @param[in] q Joint positions [rad], size = dof, for one trajectory waypoint.
   * @return Signed distance [m] at the given waypoint; negative values indicate
   * penetration if the backend reports signed distances.
   */
  double evaluateAt(const JointVecView& q) const override;

private:
  double computeValue(double distance) const;

private:
  DistanceQueryPtr query_;
  Options opt_;
};

}  // namespace rbt_features
