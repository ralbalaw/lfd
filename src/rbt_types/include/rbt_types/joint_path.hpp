#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rbt_types/math.hpp"

namespace rbt_types
{

/// @brief Finite joint positions forming a path with at least two waypoints.
class JointPath
{
public:
  /**
   * @brief Constructs a joint path.
   * @param[in] positions Joint positions [rad], shape = (num_waypoints, dof) RowMajor.
   * @throws std::invalid_argument If there are fewer than two waypoints, no joints, or
   * nonfinite values.
   */
  explicit JointPath(JointPathMat positions) : positions_(std::move(positions))
  {
    if (positions_.cols() == 0)
    {
      throw std::invalid_argument("JointPath positions must be non-empty.");
    }

    if (positions_.rows() < 2)
    {
      throw std::invalid_argument(
          "JointPath positions must contain at least two waypoints.");
    }

    if (!positions_.allFinite())
    {
      throw std::invalid_argument("JointPath positions must contain only finite values.");
    }
  }

  /**
   * @brief Returns the joint positions.
   * @return Positions [rad], shape = (length(), dim()) RowMajor; valid while this object exists.
   */
  const JointPathMat& positions() const noexcept
  {
    return positions_;
  }

  /// @brief Returns the joint dimension.
  /// @return Number of joints per waypoint.
  std::size_t dim() const noexcept
  {
    return static_cast<std::size_t>(positions_.cols());
  }

  /// @brief Returns the path length.
  /// @return Number of waypoints.
  std::size_t length() const noexcept
  {
    return static_cast<std::size_t>(positions_.rows());
  }

private:
  JointPathMat positions_;
};

/// @brief Collection of joint paths.
using JointPaths = std::vector<JointPath>;

}  // namespace rbt_types
