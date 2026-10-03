#pragma once

/**
 * @file types.hpp
 * @brief Preallocated data containers for fast ROS message conversion.
 */

#include <Eigen/Core>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace rlc_utils::types
{

/**
 * @brief Preallocated joint state container matching
 * `sensor_msgs::msg::JointState`.
 *
 * @details
 * Sizes:
 * - `name`: length = `size()` (may be left unset by converters).
 * - `position`: Joint positions [rad], size = `size()`.
 * - `velocity`: Joint velocities [rad/s], size = `size()`.
 * - `effort`: Joint effort/torque [N·m], size = `size()`.
 *
 * Timestamp:
 * - `stamp_sec`: Message stamp [s], typically derived from `header.stamp`.
 */
struct JointStateMsgData
{
  std::vector<std::string> name;
  double stamp_sec{ 0.0 };

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;

  JointStateMsgData() = default;

  /**
   * @brief Constructs a container sized for `n` joints and zeros numeric
   * fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  explicit JointStateMsgData(Eigen::Index n)
  {
    resize(n);
  }

  /**
   * @brief Resizes the container for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  void resize(Eigen::Index n)
  {
    if (n < 0)
      throw std::invalid_argument("JointStateMsgData::resize: n < 0");
    name.resize(static_cast<std::size_t>(n));

    position.setZero(n);
    velocity.setZero(n);
    effort.setZero(n);
  }

  /**
   * @brief Returns the number of joints in this container.
   * @return Joint count, equal to `position.size()`.
   */
  Eigen::Index size() const noexcept
  {
    return position.size();
  }
};

/**
 * @brief (deprecated) Preallocated joint effort command container matching
 * `rlc_interfaces::msg::JointEffortCmd`.
 *
 * @details
 * Sizes:
 * - `name`: length = `size()` (may be left unset by converters).
 * - `effort`: Joint effort/torque command [N·m], size = `size()`.
 *
 * Timestamp:
 * - `stamp_sec`: Message stamp [s], typically derived from `header.stamp`.
 */
struct JointEffortCmdMsgData
{
  std::vector<std::string> name;
  double stamp_sec{ 0.0 };
  Eigen::VectorXd effort;

  JointEffortCmdMsgData() = default;

  /**
   * @brief Constructs a container sized for `n` joints and zeros numeric
   * fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  explicit JointEffortCmdMsgData(Eigen::Index n)
  {
    resize(n);
  }

  /**
   * @brief Resizes the container for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  void resize(Eigen::Index n)
  {
    if (n < 0)
      throw std::invalid_argument("JointEffortCmdMsgData::resize: n < 0");
    name.resize(static_cast<std::size_t>(n));
    effort.setZero(n);
  }

  /**
   * @brief Returns the number of joints in this container.
   * @return Joint count, equal to `effort.size()`.
   */
  Eigen::Index size() const noexcept
  {
    return effort.size();
  }
};

/**
 * @brief Preallocated controller state container matching
 * `control_msgs::msg::JointTrajectoryControllerState`.
 *
 * @details
 * Sizes:
 * - `joint_names`: length = `size()` (may be left unset by converters).
 * - `position`, `velocity`, `acceleration`: Feedback [rad], [rad/s], [rad/s^2],
 * size = `size()`.
 * - `position_des`, `velocity_des`, `acceleration_des`: Reference [rad],
 * [rad/s], [rad/s^2], size = `size()`.
 * - `effort`: Output effort/torque [N·m], size = `size()`.
 * - `position_error`, `velocity_error`: Error terms, size = `size()`.
 *
 * Timestamp:
 * - `stamp_sec`: Message stamp [s], typically derived from `header.stamp`.
 */
struct JointControllerStateMsgData
{
  std::vector<std::string> joint_names;
  double stamp_sec{ 0.0 };

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;

  Eigen::VectorXd position_des;
  Eigen::VectorXd velocity_des;
  Eigen::VectorXd acceleration_des;

  Eigen::VectorXd effort;

  Eigen::VectorXd position_error;
  Eigen::VectorXd velocity_error;

  JointControllerStateMsgData() = default;

  /**
   * @brief Constructs a container sized for `n` joints and zeros numeric
   * fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  explicit JointControllerStateMsgData(Eigen::Index n)
  {
    resize(n);
  }

  /**
   * @brief Resizes the container for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  void resize(Eigen::Index n)
  {
    if (n < 0)
      throw std::invalid_argument("JointControllerStateMsgData::resize: n < 0");
    joint_names.resize(static_cast<std::size_t>(n));

    position.setZero(n);
    velocity.setZero(n);
    acceleration.setZero(n);
    position_des.setZero(n);
    velocity_des.setZero(n);
    acceleration_des.setZero(n);
    effort.setZero(n);
    position_error.setZero(n);
    velocity_error.setZero(n);
  }

  /**
   * @brief Returns the number of joints in this container.
   * @return Joint count, equal to `position.size()`.
   */
  Eigen::Index size() const noexcept
  {
    return position.size();
  }

  /**
   * @brief Recomputes `position_error` and `velocity_error` from current and
   * desired values.
   * @details Computes:
   * - `position_error = position_des - position`
   * - `velocity_error = velocity_des - velocity`
   */
  void recompute_errors()
  {
    position_error.noalias() = position_des - position;
    velocity_error.noalias() = velocity_des - velocity;
  }
};

/**
 * @brief Preallocated container mirroring
 * `trajectory_msgs::msg::JointTrajectory`.
 *
 * @details
 * This struct is intended as an internal, preallocated representation suitable
 * for real-time control loops after conversion from ROS messages in a
 * non-real-time context.
 *
 * Message fields covered:
 * - Header:
 *   - `stamp_sec`: Header stamp [s], typically derived from `header.stamp`.
 *   - `frame_id`: Header frame id string.
 * - Trajectory:
 *   - `joint_names`: Joint name list, length = `n()`.
 *   - `t`: Time-from-start [s], shape = (N,).
 *   - `q`: Positions [rad], shape = (N, n()) RowMajor.
 *   - `qd`: Velocities [rad/s], shape = (N, n()) RowMajor.
 *   - `qdd`: Accelerations [rad/s^2], shape = (N, n()) RowMajor.
 *   - `effort`: Effort/torque [N·m], shape = (N, n()) RowMajor.
 *
 * Optional point fields:
 * - In ROS, `velocities`, `accelerations`, and `effort` may be empty per-point.
 *   When converting, set `has_qd/has_qdd/has_effort` accordingly and either:
 *   - leave the corresponding matrix sized and filled with zeros, or
 *   - size it to (0, 0) if you prefer sparse storage (not recommended for RT).
 */
struct JointTrajectoryMsgData
{
  using MatTraj = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  std::vector<std::string> joint_names;
  uint64_t seq{ 0 };

  Eigen::VectorXd t;

  MatTraj q;
  MatTraj qd;
  MatTraj qdd;
  MatTraj effort;

  JointTrajectoryMsgData() = default;

  /**
   * @brief Resizes buffers for `N` samples and `n` joints and zeros numeric
   * fields.
   * @param[in] N Number of trajectory samples (points).
   * @param[in] n Number of joints.
   * @throws std::invalid_argument If `N < 0` or `n < 0`.
   */
  void resize(std::size_t N, std::size_t n)
  {
    t.setZero(N);
    q.setZero(N, n);
    qd.setZero(N, n);
    qdd.setZero(N, n);
    effort.setZero(N, n);
  }

  /**
   * @brief Returns the number of trajectory samples.
   * @return Sample count `N`.
   */
  std::size_t length() const noexcept
  {
    return static_cast<std::size_t>(t.size());
  }
  std::size_t ndof() const noexcept
  {
    return static_cast<std::size_t>(joint_names.size());
  }
};
}  // namespace rlc_utils::types
