#pragma once

#include <string>
#include <vector>

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "rbt_types/joint_path.hpp"
#include "rbt_types/joint_trajectory.hpp"

namespace rlc_executive
{
namespace bt_utils
{

/**
 * @brief Converts a MoveIt robot trajectory to a joint trajectory.
 * @param[in] robot_trajectory MoveIt joint trajectory with waypoint positions and
 * timing.
 * @return Joint trajectory preserving every waypoint time.
 * @throws std::invalid_argument If positions, names, or times are invalid.
 */
rbt_types::JointTrajectory
fromRobotTrajectory(const moveit_msgs::msg::RobotTrajectory& robot_trajectory);

/**
 * @brief Converts a joint path to an untimed MoveIt robot trajectory.
 * @param[in] path Joint path.
 * @param[in] joint_names Joint names matching path column order, size = `path.dim()`;
 * may be empty.
 * @return MoveIt robot trajectory containing path positions.
 * @throws std::invalid_argument If `joint_names` has the wrong size.
 */
moveit_msgs::msg::RobotTrajectory
toRobotTrajectory(const rbt_types::JointPath& path,
                  const std::vector<std::string>& joint_names);

/**
 * @brief Converts a joint trajectory to a timed MoveIt robot trajectory.
 * @param[in] joint_trajectory Joint trajectory.
 * @param[in] joint_names Joint names matching trajectory joint order, size =
 * `joint_trajectory.joint_names.size()`; may be empty.
 * @return MoveIt robot trajectory containing path positions.
 * @throws std::invalid_argument If `joint_names` has the wrong size.
 */
moveit_msgs::msg::RobotTrajectory
toRobotTrajectory(const rbt_types::JointTrajectory& joint_trajectory,
                  const std::vector<std::string>& joint_names);

}  // namespace bt_utils
}  // namespace rlc_executive
