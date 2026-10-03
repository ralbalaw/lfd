#include "rlc_utils/msg_conv.hpp"

#include <Eigen/src/Core/util/Meta.h>

#include <Eigen/Core>
#include <cstring>

#include "rlc_utils/checks.hpp"
#include "rlc_utils/time_util.hpp"

namespace ru = rlc_utils;
namespace rutime = rlc_utils::time_util;
namespace rutypes = rlc_utils::types;

namespace rlc_utils::msg_conv
{

sensor_msgs::msg::JointState make_joint_state_msg(std::size_t n, double stamp_sec,
                                                  const std::vector<std::string>& names,
                                                  bool include_effort)
{
  sensor_msgs::msg::JointState msg;

  msg.name = names;
  msg.header.stamp = rutime::to_ros_time(stamp_sec);

  msg.position.resize(n);
  msg.velocity.resize(n);

  if (include_effort)
  {
    msg.effort.resize(n);
  }
  else
  {
    msg.effort.clear();
  }

  return msg;
}

control_msgs::msg::JointTrajectoryControllerState
make_joint_ctrl_state_msg(std::size_t n, double stamp_sec,
                          const std::vector<std::string>& names,
                          bool include_feedback_acceleration)
{
  control_msgs::msg::JointTrajectoryControllerState msg;

  msg.joint_names = names;
  msg.header.stamp = rutime::to_ros_time(stamp_sec);

  msg.reference.positions.resize(n);
  msg.feedback.positions.resize(n);
  msg.output.effort.resize(n);
  msg.reference.velocities.resize(n);
  msg.feedback.velocities.resize(n);
  msg.reference.accelerations.resize(n);
  msg.error.velocities.resize(n);
  msg.error.positions.resize(n);

  if (include_feedback_acceleration)
  {
    msg.feedback.accelerations.resize(n);
  }
  else
  {
    msg.feedback.accelerations.clear();
  }

  return msg;
}

bool from_joint_state_msg(const sensor_msgs::msg::JointState& msg,
                          rutypes::JointStateMsgData& data) noexcept
{
  const Eigen::Index n = data.size();
  const auto bytes = static_cast<std::size_t>(n) * sizeof(double);

  if (!ru::checks::size_is(msg.position, n))
    return false;
  if (!ru::checks::empty_or_size_is(msg.name, n))
    return false;
  if (!ru::checks::empty_or_size_is(msg.velocity, n))
    return false;
  if (!ru::checks::empty_or_size_is(msg.effort, n))
    return false;

  data.stamp_sec = rutime::from_ros_time(msg.header.stamp);

  std::memcpy(data.position.data(), msg.position.data(), bytes);

  if (!msg.velocity.empty())
  {
    std::memcpy(data.velocity.data(), msg.velocity.data(), bytes);
  }
  else
  {
    data.velocity.setZero();
  }

  if (!msg.effort.empty())
  {
    std::memcpy(data.effort.data(), msg.effort.data(), bytes);
  }
  else
  {
    data.effort.setZero();
  }

  return true;
}

bool from_joint_ctrl_state_msg(const control_msgs::msg::JointTrajectoryControllerState& msg,
                               rutypes::JointControllerStateMsgData& data) noexcept
{
  const Eigen::Index n = data.size();
  const auto bytes = static_cast<std::size_t>(n) * sizeof(double);

  if (!ru::checks::empty_or_size_is(msg.joint_names, n))
    return false;

  if (!ru::checks::all_size_is(n, msg.feedback.positions, msg.feedback.velocities,
                               msg.reference.positions, msg.reference.velocities,
                               msg.reference.accelerations, msg.error.positions,
                               msg.error.velocities, msg.output.effort))
  {
    return false;
  }

  if (!ru::checks::empty_or_size_is(msg.feedback.accelerations, n))
    return false;

  data.stamp_sec = rutime::from_ros_time(msg.header.stamp);

  std::memcpy(data.position.data(), msg.feedback.positions.data(), bytes);
  std::memcpy(data.velocity.data(), msg.feedback.velocities.data(), bytes);
  std::memcpy(data.position_des.data(), msg.reference.positions.data(), bytes);
  std::memcpy(data.velocity_des.data(), msg.reference.velocities.data(), bytes);
  std::memcpy(data.acceleration_des.data(), msg.reference.accelerations.data(), bytes);
  std::memcpy(data.effort.data(), msg.output.effort.data(), bytes);
  std::memcpy(data.position_error.data(), msg.error.positions.data(), bytes);
  std::memcpy(data.velocity_error.data(), msg.error.velocities.data(), bytes);

  if (!msg.feedback.accelerations.empty())
  {
    std::memcpy(data.acceleration.data(), msg.feedback.accelerations.data(), bytes);
  }
  else
  {
    data.acceleration.setZero();
  }

  return true;
}

bool to_joint_state_msg(const rutypes::JointStateMsgData& data,
                        sensor_msgs::msg::JointState& msg) noexcept
{
  const Eigen::Index n = data.size();
  const auto bytes = static_cast<std::size_t>(n) * sizeof(double);

  msg.header.stamp = rutime::to_ros_time(data.stamp_sec);

  std::memcpy(msg.position.data(), data.position.data(), bytes);
  std::memcpy(msg.velocity.data(), data.velocity.data(), bytes);

  if (!msg.effort.empty())
  {
    std::memcpy(msg.effort.data(), data.effort.data(), bytes);
  }
  return true;
}

bool to_joint_ctrl_state_msg(const rutypes::JointControllerStateMsgData& data,
                             control_msgs::msg::JointTrajectoryControllerState& msg) noexcept
{
  const Eigen::Index n = data.size();
  const auto bytes = static_cast<std::size_t>(n) * sizeof(double);

  msg.header.stamp = rutime::to_ros_time(data.stamp_sec);

  std::memcpy(msg.feedback.positions.data(), data.position.data(), bytes);
  std::memcpy(msg.feedback.velocities.data(), data.velocity.data(), bytes);
  std::memcpy(msg.reference.positions.data(), data.position_des.data(), bytes);
  std::memcpy(msg.reference.velocities.data(), data.velocity_des.data(), bytes);
  std::memcpy(msg.reference.accelerations.data(), data.acceleration_des.data(), bytes);
  std::memcpy(msg.error.positions.data(), data.position_error.data(), bytes);
  std::memcpy(msg.error.velocities.data(), data.velocity_error.data(), bytes);
  std::memcpy(msg.output.effort.data(), data.effort.data(), bytes);

  if (!msg.feedback.accelerations.empty())
  {
    std::memcpy(msg.feedback.accelerations.data(), data.acceleration.data(), bytes);
  }
  return true;
}

bool from_joint_trajectory_msg(const trajectory_msgs::msg::JointTrajectory& msg,
                               rutypes::JointTrajectoryMsgData& data) noexcept
{
  const std::size_t n = data.ndof();
  const std::size_t N = data.length();
  const auto bytes = n * sizeof(double);

  if (n <= 0 || N <= 0)
    return false;

  if (!ru::checks::size_is(msg.joint_names, n))
    return false;
  if (!ru::checks::size_is(data.joint_names, n))
    return false;
  for (std::size_t i = 0; i < n; ++i)
  {
    if (msg.joint_names[i] != data.joint_names[i])
      return false;
  }

  if (!ru::checks::size_is(msg.points, N))
    return false;

  for (std::size_t k = 0; k < N; ++k)
  {
    const auto& pt = msg.points[k];
    const double t = rutime::from_ros_duration(pt.time_from_start);

    data.t(static_cast<Eigen::Index>(k)) = t;

    if (!ru::checks::size_is(pt.positions, n))
      return false;
    if (!ru::checks::size_is(pt.velocities, n))
      return false;
    if (!ru::checks::size_is(pt.accelerations, n))
      return false;

    const auto row_off = k * n;

    std::memcpy(data.q.data() + row_off, pt.positions.data(), bytes);
    std::memcpy(data.qd.data() + row_off, pt.velocities.data(), bytes);
    std::memcpy(data.qdd.data() + row_off, pt.accelerations.data(), bytes);
  }

  return true;
}

}  // namespace rlc_utils::msg_conv
