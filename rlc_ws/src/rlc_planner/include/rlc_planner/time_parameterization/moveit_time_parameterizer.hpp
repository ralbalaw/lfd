#pragma once

/**
 * @file moveit_time_parameterizer.hpp
 * @brief MoveIt-backed time-parameterization utility for MoveIt trajectories.
 */

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_model/robot_model.hpp>

#include <rlc_interfaces/srv/time_parameterize_trajectory.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include <memory>

namespace robot_model_loader
{
class RobotModelLoader;
}  // namespace robot_model_loader

namespace rlc_planner
{

/**
 * @brief Converts an untimed MoveIt trajectory into a timed trajectory.
 *
 * @details
 * The supplied `robot_trajectory::RobotTrajectory` must already be associated with the
 * correct MoveIt robot model and joint model group. This class only runs MoveIt's
 * `TimeOptimalTrajectoryGeneration` and serializes the timed result back to a ROS
 * message.
 */
class MoveItTimeParameterizer final : public rclcpp::Node
{
public:
  using TimeParameterizeTrajectory = rlc_interfaces::srv::TimeParameterizeTrajectory;

  /**
   * @brief Constructs the node with the fixed node name
   * `moveit_time_parameterizer`.
   */
  MoveItTimeParameterizer();

  /**
   * @brief Creates and initializes the service node.
   * @return Shared node instance ready to serve `time_parameterize_trajectory`
   * requests.
   */
  static std::shared_ptr<MoveItTimeParameterizer> create();

  /**
   * @brief Loads the MoveIt robot model and advertises the service.
   * @throws std::runtime_error If the MoveIt robot model cannot be loaded from the
   * local `robot_description*` parameters.
   */
  void initialize();

  /**
   * @brief Time-parameterizes an untimed MoveIt trajectory.
   * @param[in] trajectory Untimed MoveIt trajectory containing at least two waypoints and
   * already bound to the intended MoveIt joint model group.
   * @param[in] max_velocity_scaling_factor Velocity scaling factor in the range `[0, 1]`
   * passed directly to MoveIt's TimeOptimalTrajectoryGeneration.
   * @param[in] max_acceleration_scaling_factor Acceleration scaling factor in the range
   * `[0, 1]` passed directly to MoveIt's TimeOptimalTrajectoryGeneration.
   * @return Timed MoveIt robot trajectory message for the trajectory's current group.
   * @throws std::runtime_error If `trajectory` is not associated with a MoveIt joint
   * model group, contains fewer than two waypoints, or MoveIt's time parameterization
   * fails.
   */
  moveit_msgs::msg::RobotTrajectory
  parameterize(robot_trajectory::RobotTrajectory trajectory,
               double max_velocity_scaling_factor,
               double max_acceleration_scaling_factor) const;

private:
  /**
   * @brief Handles one time-parameterization service request.
   * @param[in] request Service request containing the untimed path and scaling factors.
   * @param[out] response Service response populated with a timed trajectory or an error.
   */
  void handleRequest(const TimeParameterizeTrajectory::Request::SharedPtr& request,
                     const TimeParameterizeTrajectory::Response::SharedPtr& response);

  std::shared_ptr<robot_model_loader::RobotModelLoader>
      robot_model_loader_;  ///< Loader that owns MoveIt robot model resources.
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Service<TimeParameterizeTrajectory>::SharedPtr
      service_;  ///< Advertised ROS service handle.
};

}  // namespace rlc_planner
