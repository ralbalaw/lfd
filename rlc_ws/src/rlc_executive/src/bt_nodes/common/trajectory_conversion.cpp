#include "rlc_executive/bt_nodes/common/trajectory_conversion.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

#include <rclcpp/duration.hpp>

namespace rlc_executive
{
namespace bt_utils
{

rbt_types::JointTrajectory
fromRobotTrajectory(const moveit_msgs::msg::RobotTrajectory& robot_trajectory)
{
  const auto& joint_trajectory = robot_trajectory.joint_trajectory;
  const auto& points = joint_trajectory.points;
  if (points.size() < 2U)
  {
    throw std::invalid_argument("robot trajectory must contain at least two waypoints");
  }

  const std::size_t num_joints = points.front().positions.size();
  if (num_joints == 0U)
  {
    throw std::invalid_argument("robot trajectory positions must be non-empty");
  }
  if (!joint_trajectory.joint_names.empty() &&
      joint_trajectory.joint_names.size() != num_joints)
  {
    throw std::invalid_argument(
        "robot trajectory joint names must match the position dimension");
  }

  rbt_types::JointPathMat positions(static_cast<Eigen::Index>(points.size()),
                                    static_cast<Eigen::Index>(num_joints));
  rbt_types::TimeVec time_from_start(static_cast<Eigen::Index>(points.size()));

  for (std::size_t row = 0; row < points.size(); ++row)
  {
    if (points[row].positions.size() != num_joints)
    {
      throw std::invalid_argument(
          "robot trajectory waypoint position dimensions must match");
    }

    for (std::size_t col = 0; col < num_joints; ++col)
    {
      positions(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          points[row].positions[col];
    }
    time_from_start[static_cast<Eigen::Index>(row)] =
        rclcpp::Duration(points[row].time_from_start).seconds();
  }

  return rbt_types::JointTrajectory(std::move(positions), std::move(time_from_start));
}

moveit_msgs::msg::RobotTrajectory
toRobotTrajectory(const rbt_types::JointPath& path,
                  const std::vector<std::string>& joint_names)
{
  const std::size_t num_points = path.length();
  const std::size_t num_joints = path.dim();

  if (!joint_names.empty() && joint_names.size() != num_joints)
  {
    throw std::invalid_argument("joint names must match the trajectory dimension");
  }

  const auto& positions = path.positions();

  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  auto& joint_trajectory_msg = robot_trajectory_msg.joint_trajectory;

  joint_trajectory_msg.joint_names = joint_names;
  joint_trajectory_msg.points.resize(num_points);

  for (std::size_t row = 0; row < num_points; ++row)
  {
    auto& point = joint_trajectory_msg.points[row];
    const Eigen::Index eigen_row = static_cast<Eigen::Index>(row);

    point.positions.resize(num_joints);

    const double* source = positions.data() + eigen_row * positions.cols();

    std::copy_n(source, num_joints, point.positions.data());
  }

  return robot_trajectory_msg;
}

moveit_msgs::msg::RobotTrajectory
toRobotTrajectory(const rbt_types::JointTrajectory& joint_trajectory,
                  const std::vector<std::string>& joint_names)
{
  const std::size_t num_points = joint_trajectory.length();
  const std::size_t num_joints = joint_trajectory.dim();

  if (!joint_names.empty() && joint_names.size() != num_joints)
  {
    throw std::invalid_argument("joint names must match the trajectory dimension");
  }

  const auto& positions = joint_trajectory.positions();
  const auto& time_from_start = joint_trajectory.timeFromStart();

  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  auto& joint_trajectory_msg = robot_trajectory_msg.joint_trajectory;

  joint_trajectory_msg.joint_names = joint_names;
  joint_trajectory_msg.points.resize(num_points);

  for (std::size_t row = 0; row < num_points; ++row)
  {
    auto& point = joint_trajectory_msg.points[row];
    const Eigen::Index eigen_row = static_cast<Eigen::Index>(row);

    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start[eigen_row]);

    point.positions.resize(num_joints);

    const double* source = positions.data() + eigen_row * positions.cols();

    std::copy_n(source, num_joints, point.positions.data());
  }

  return robot_trajectory_msg;
}

}  // namespace bt_utils
}  // namespace rlc_executive
