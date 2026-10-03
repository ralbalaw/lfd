#pragma once

/**
 * @file time_util.hpp
 * @brief Conversion helpers between ROS time/duration messages and seconds.
 */

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace rlc_utils::time_util {

/**
 * @brief Converts a ROS `Time` message to seconds.
 * @param[in] t ROS time.
 * @return Time [s] as `t.sec + 1e-9 * t.nanosec`.
 */
double from_ros_time(const builtin_interfaces::msg::Time& t) noexcept;

/**
 * @brief Converts a ROS `Duration` message to seconds.
 * @param[in] d ROS duration.
 * @return Duration [s] as `d.sec + 1e-9 * d.nanosec`.
 */
double from_ros_duration(const builtin_interfaces::msg::Duration& d) noexcept;

/**
 * @brief Converts seconds to a ROS `Time` message.
 * @param[in] sec Time [s].
 * @return ROS time message with `nanosec` in `[0, 999999999]`.
 *
 * @details
 * - Non-finite or negative inputs are clamped to `0.0`.
 * - Fractional seconds are rounded to the nearest nanosecond with carry into `sec`.
 * - Output seconds are clamped to `[0, INT32_MAX]` to match the ROS message range.
 */
builtin_interfaces::msg::Time to_ros_time(double sec) noexcept;

/**
 * @brief Converts seconds to a ROS `Duration` message.
 * @param[in] sec Duration [s].
 * @return ROS duration message with `nanosec` in `[0, 999999999]`.
 *
 * @details
 * - Non-finite or negative inputs are clamped to `0.0` (no negative durations are produced).
 * - Fractional seconds are rounded to the nearest nanosecond with carry into `sec`.
 * - Output seconds are clamped to `[INT32_MIN, INT32_MAX]` to match the ROS message range.
 */
builtin_interfaces::msg::Duration to_ros_duration(double sec) noexcept;

}  // namespace rlc_utils::time_util
