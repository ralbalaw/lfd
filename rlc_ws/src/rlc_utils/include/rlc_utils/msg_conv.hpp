#pragma once

/**
 * @file msg_conv.hpp
 * @brief Allocation-free conversions between ROS messages and preallocated data
 * containers.
 */

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "rlc_utils/types.hpp"

namespace rlc_utils::msg_conv
{

/**
 * @brief Creates a `sensor_msgs::msg::JointState` message with arrays sized for
 * `n` joints.
 * @param[in] n Joint count (array length).
 * @param[in] stamp_sec Message stamp [s].
 * @param[in] names Optional joint names, length = `n`; may be empty.
 * @param[in] include_effort If true, allocate `effort` with length = `n`;
 * otherwise leave it empty.
 * @return JointState message with `position` and `velocity` length = `n`, and
 * `effort` optionally length = `n`.
 * @note This function does not validate `names.size()`; pass `names` empty or
 * length = `n`.
 */
sensor_msgs::msg::JointState
make_joint_state_msg(std::size_t n, double stamp_sec = 0.0,
                     const std::vector<std::string>& names = {},
                     bool include_effort = true);

/**
 * @brief Creates a `control_msgs::msg::JointTrajectoryControllerState` message
 * with arrays sized for `n` joints.
 * @param[in] n Joint count (array length).
 * @param[in] stamp_sec Message stamp [s].
 * @param[in] names Optional joint names, length = `n`; may be empty.
 * @param[in] include_feedback_acceleration If true, allocate
 * `feedback.accelerations` with length = `n`.
 * @return JointTrajectoryControllerState message with required arrays allocated
 * to length = `n`.
 * @note This function does not validate `names.size()`; pass `names` empty or
 * length = `n`.
 */
control_msgs::msg::JointTrajectoryControllerState
make_joint_ctrl_state_msg(std::size_t n, double stamp_sec = 0.0,
                          const std::vector<std::string>& names = {},
                          bool include_feedback_acceleration = false);

/**
 * @brief Copies a ROS `JointState` message into a preallocated data container.
 *
 * Copies joint arrays from @p msg into @p data without resizing @p data.
 *
 * @param[in] msg Source message.
 * @param[out] data Destination container (preallocated).
 * @return True on success; false if any required/optional field violates the
 * size contract.
 *
 * @details
 * Contract (with `n = data.size()`):
 * - `msg.position.size()` must equal `n`.
 * - `msg.name`, `msg.velocity`, `msg.effort` may be empty; if non-empty, each
 * must have size = `n`.
 * - If `msg.velocity` is empty, `data.velocity` is set to zero.
 * - If `msg.effort` is empty, `data.effort` is set to zero.
 *
 * Sets `data.stamp_sec` from `msg.header.stamp`. Joint names are not copied.
 *
 * @note Allocation-free; O(n) in the number of joints (copies via
 * `std::memcpy`).
 */
[[nodiscard]] bool from_joint_state_msg(const sensor_msgs::msg::JointState& msg,
                                        types::JointStateMsgData& data) noexcept;

/**
 * @brief Copies a ROS `JointTrajectoryControllerState` message into a
 * preallocated data container.
 * @param[in] msg Source message.
 * @param[out] data Destination container (preallocated).
 * @return True on success; false if any required/optional field violates the
 * size contract.
 *
 * @details
 * Contract (with `n = data.size()`):
 * - `msg.joint_names` may be empty; if non-empty, it must have length = `n`.
 * - `feedback.positions`, `feedback.velocities`, `reference.positions`,
 * `reference.velocities`, `reference.accelerations`, `error.positions`,
 * `error.velocities`, and `output.effort` must all have length = `n`.
 * - `feedback.accelerations` may be empty; if empty, `data.acceleration` is set
 * to zero.
 *
 * Sets `data.stamp_sec` from `msg.header.stamp`. Joint names are not copied.
 *
 * @note Allocation-free; O(n) in the number of joints (copies via
 * `std::memcpy`).
 */
[[nodiscard]] bool
from_joint_ctrl_state_msg(const control_msgs::msg::JointTrajectoryControllerState& msg,
                          types::JointControllerStateMsgData& data) noexcept;

/**
 * @brief Copies a ROS `JointTrajectory` message into a preallocated data
 * container.
 * @param[in] msg Source message.
 * @param[out] data Destination container (preallocated).
 * @return True on success; false if any required/optional field violates the
 * size contract.
 *
 * @details
 * Contract (with `n = data.ndof()`, `N = data.length()`):
 * - `msg.joint_names` must have length = `n`.
 * - `msg.points` must have length = `N`.
 * - For each point in `msg.points`:
 *   - `positions` and `velocities` must have length = `n`.
 *
 * Sets `data.joint_names` from `msg.joint_names` and fills in `data.t`,
 * `data.q`, and `data.qd` from the message points. Accelerations and effort are
 * not processed.
 *
 * @note Allocation-free; O(N·n) in the number of trajectory points and joints
 * (copies via `std::memcpy`).
 */
[[nodiscard]] bool
from_joint_trajectory_msg(const trajectory_msgs::msg::JointTrajectory& msg,
                          types::JointTrajectoryMsgData& data) noexcept;

/**
 * @brief Copies a preallocated data container into a ROS `JointState` message.
 * @param[in] data Source container.
 * @param[out] msg Destination message (preallocated arrays).
 * @return True.
 *
 * @pre `msg.position.size() == data.size()`
 * @pre `msg.velocity.size() == data.size()`
 * @pre `msg.effort.empty() || msg.effort.size() == data.size()`
 *
 * Sets `msg.header.stamp` from `data.stamp_sec`.
 *
 * @details
 * This function does not resize `msg` fields; it performs allocation-free
 * copies via `std::memcpy`. Use `make_joint_state_msg()` to construct a
 * correctly sized `msg`.
 *
 * @note `msg.name` is not modified; if non-empty, keep it aligned with the
 * joint arrays.
 *
 * @note Allocation-free; O(n) in the number of joints (copies via
 * `std::memcpy`).
 */
bool to_joint_state_msg(const types::JointStateMsgData& data,
                        sensor_msgs::msg::JointState& msg) noexcept;

/**
 * @brief Copies a preallocated data container into a ROS
 * `JointTrajectoryControllerState` message.
 * @param[in] data Source container.
 * @param[out] msg Destination message (preallocated arrays).
 * @return True.
 *
 * @pre `msg.feedback.positions.size() == data.size()`
 * @pre `msg.feedback.velocities.size() == data.size()`
 * @pre `msg.reference.positions.size() == data.size()`
 * @pre `msg.reference.velocities.size() == data.size()`
 * @pre `msg.reference.accelerations.size() == data.size()`
 * @pre `msg.error.positions.size() == data.size()`
 * @pre `msg.error.velocities.size() == data.size()`
 * @pre `msg.output.effort.size() == data.size()`
 * @pre `msg.feedback.accelerations.empty() || msg.feedback.accelerations.size() == data.size()`
 *
 * Sets `msg.header.stamp` from `data.stamp_sec`.
 *
 * @details
 * This function does not resize `msg` fields; it performs allocation-free
 * copies via `std::memcpy`. Use `make_joint_ctrl_state_msg()` to construct a
 * correctly sized `msg`.
 *
 * @note `msg.joint_names` is not modified; if non-empty, keep it aligned with
 * the joint arrays.
 *
 * @note Allocation-free; O(n) in the number of joints (copies via
 * `std::memcpy`).
 */
bool to_joint_ctrl_state_msg(
    const types::JointControllerStateMsgData& data,
    control_msgs::msg::JointTrajectoryControllerState& msg) noexcept;

}  // namespace rlc_utils::msg_conv
