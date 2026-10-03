#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rbt_types/math.hpp"

namespace rbt_types
{

/// @brief Interface for evaluating a scalar feature on joint positions or paths.
class Feature
{
public:
  /// @brief Destroys the feature.
  virtual ~Feature() = default;

  /**
   * @brief Returns the feature name.
   * @return Human-readable feature name.
   */
  virtual std::string name() const = 0;

  /**
   * @brief Evaluates the feature over a joint path.
   * @param[in] path Joint positions [rad], shape = (num_waypoints, dof)
   * RowMajor; the view is not stored.
   * @return Scalar feature value for `path`.
   */
  virtual double evaluate(const JointPathView& path) const = 0;

  /**
   * @brief Evaluates the feature at one joint position.
   * @param[in] joint_positions Joint positions [rad], size = dof; the view is not stored.
   * @return Scalar feature value at `joint_positions`.
   */
  virtual double evaluateAt(const JointVecView& joint_positions) const = 0;
};

/// @brief Shared immutable feature implementation.
using FeaturePtr = std::shared_ptr<const Feature>;

/// @brief Collection of immutable feature implementations.
using Features = std::vector<FeaturePtr>;

}  // namespace rbt_types
