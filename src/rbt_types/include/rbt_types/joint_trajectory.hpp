#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rbt_types/joint_path.hpp"
#include "rbt_types/math.hpp"

namespace rbt_types
{

/// @brief Joint path with finite, nonnegative, strictly increasing waypoint times.
class JointTrajectory
{
public:
  /**
   * @brief Constructs a joint trajectory.
   * @param[in] positions Joint positions [rad], shape = (num_waypoints, dof) RowMajor.
   * @param[in] time_from_start Strictly increasing waypoint times [s], size = num_waypoints.
   * @throws std::invalid_argument If the positions or times are invalid.
   */
  JointTrajectory(JointPathMat positions, TimeVec time_from_start)
    : JointTrajectory(JointPath{ std::move(positions) }, std::move(time_from_start))
  {
  }

  /**
   * @brief Constructs a timed joint path.
   * @param[in] path Joint path.
   * @param[in] time_from_start Strictly increasing waypoint times [s], size = `path.length()`.
   * @throws std::invalid_argument If the times are invalid.
   */
  JointTrajectory(JointPath path, TimeVec time_from_start)
    : path_(std::move(path)), time_from_start_(std::move(time_from_start))
  {
    validateTime();
  }

  /**
   * @brief Returns the joint positions.
   * @return Positions [rad], shape = (length(), dim()) RowMajor; valid while this object exists.
   */
  const JointPathMat& positions() const noexcept
  {
    return path_.positions();
  }

  /**
   * @brief Returns the joint path.
   * @return Path; valid while this object exists.
   */
  const JointPath& path() const noexcept
  {
    return path_;
  }

  /**
   * @brief Returns the waypoint times.
   * @return Times from start [s], size = length(); valid while this object exists.
   */
  const TimeVec& timeFromStart() const noexcept
  {
    return time_from_start_;
  }

  /// @brief Returns the joint dimension.
  /// @return Number of joints per waypoint.
  std::size_t dim() const noexcept
  {
    return path_.dim();
  }

  /// @brief Returns the trajectory length.
  /// @return Number of waypoints.
  std::size_t length() const noexcept
  {
    return path_.length();
  }

private:
  void validateTime() const
  {
    if (time_from_start_.size() != path_.length())
    {
      throw std::invalid_argument(
          "JointTrajectory times must match the number of waypoints.");
    }

    if (!time_from_start_.allFinite())
    {
      throw std::invalid_argument(
          "JointTrajectory times must contain only finite values.");
    }

    if ((time_from_start_.array() < 0.0).any())
    {
      throw std::invalid_argument("JointTrajectory times must be nonnegative.");
    }

    for (Eigen::Index i = 1; i < time_from_start_.size(); ++i)
    {
      if (!(time_from_start_[i] > time_from_start_[i - 1]))
      {
        throw std::invalid_argument("JointTrajectory times must be strictly increasing.");
      }
    }
  }

  JointPath path_;
  TimeVec time_from_start_;
};

/// @brief Collection of joint trajectories.
using JointTrajectories = std::vector<JointTrajectory>;

}  // namespace rbt_types
