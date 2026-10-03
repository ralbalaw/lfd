#include <rlc_planner/time_parameterization/moveit_time_parameterizer.hpp>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <stdexcept>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace rlc_planner
{
namespace
{
moveit::core::RobotState
buildReferenceState(const moveit::core::RobotModelConstPtr& robot_model,
                    const sensor_msgs::msg::JointState& reference_joint_state)
{
  if (reference_joint_state.name.size() != reference_joint_state.position.size())
  {
    throw std::runtime_error("reference_joint_state.name and "
                             "reference_joint_state.position must have the same size");
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();

  if (!reference_joint_state.name.empty())
  {
    state.setVariableValues(reference_joint_state);
  }

  state.update();
  return state;
}

robot_trajectory::RobotTrajectory
buildRobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                     const std::string& group_name,
                     const moveit_msgs::msg::RobotTrajectory& path,
                     const sensor_msgs::msg::JointState& reference_joint_state)
{
  robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);
  if (trajectory.getGroup() == nullptr)
  {
    throw std::runtime_error("unknown MoveIt joint model group: '" + group_name + "'");
  }

  if (path.joint_trajectory.points.size() < 2)
  {
    throw std::runtime_error("path must contain at least two joint waypoints");
  }

  moveit_msgs::msg::RobotTrajectory path_with_names = path;
  if (path_with_names.joint_trajectory.joint_names.empty())
  {
    path_with_names.joint_trajectory.joint_names = trajectory.getGroup()->getVariableNames();
  }

  const moveit::core::RobotState reference_state =
      buildReferenceState(robot_model, reference_joint_state);
  trajectory.setRobotTrajectoryMsg(reference_state, path_with_names);
  return trajectory;
}

}  // namespace

MoveItTimeParameterizer::MoveItTimeParameterizer()
  : rclcpp::Node("moveit_time_parameterizer")
{
}

std::shared_ptr<MoveItTimeParameterizer> MoveItTimeParameterizer::create()
{
  auto node = std::make_shared<MoveItTimeParameterizer>();
  node->initialize();
  return node;
}

void MoveItTimeParameterizer::initialize()
{
  robot_model_loader::RobotModelLoader::Options options("robot_description");
  options.load_kinematics_solvers = false;

  robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), options);

  robot_model_ = robot_model_loader_->getModel();
  if (robot_model_ == nullptr)
  {
    throw std::runtime_error(
        "Failed to load MoveIt robot model from local robot_description parameters");
  }

  service_ = create_service<TimeParameterizeTrajectory>(
      "time_parameterize_trajectory",
      [this](const TimeParameterizeTrajectory::Request::SharedPtr& request,
             const TimeParameterizeTrajectory::Response::SharedPtr& response) {
        handleRequest(request, response);
      });
}

void MoveItTimeParameterizer::handleRequest(
    const TimeParameterizeTrajectory::Request::SharedPtr& request,
    const TimeParameterizeTrajectory::Response::SharedPtr& response)
{
  response->success = false;
  response->trajectory = moveit_msgs::msg::RobotTrajectory{};
  response->error.clear();

  try
  {
    auto trajectory = buildRobotTrajectory(robot_model_, request->group_name,
                                           request->path, request->reference_joint_state);
    response->trajectory = parameterize(trajectory, request->max_velocity_scaling_factor,
                                        request->max_acceleration_scaling_factor);
    response->success = true;
  }
  catch (const std::exception& ex)
  {
    response->error = ex.what();
    RCLCPP_ERROR(get_logger(), "time_parameterize_trajectory failed: %s",
                 response->error.c_str());
  }
}

moveit_msgs::msg::RobotTrajectory
MoveItTimeParameterizer::parameterize(robot_trajectory::RobotTrajectory trajectory,
                                      double max_velocity_scaling_factor,
                                      double max_acceleration_scaling_factor) const
{
  if (trajectory.getGroup() == nullptr)
  {
    throw std::runtime_error("MoveItTimeParameterizer requires a RobotTrajectory with a "
                             "valid joint model group");
  }
  if (trajectory.getWayPointCount() < 2)
  {
    throw std::runtime_error(
        "MoveItTimeParameterizer requires a RobotTrajectory with at least two waypoints");
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(trajectory, max_velocity_scaling_factor,
                              max_acceleration_scaling_factor))
  {
    throw std::runtime_error(
        "MoveIt TimeOptimalTrajectoryGeneration failed to time-parameterize the path");
  }

  moveit_msgs::msg::RobotTrajectory output;
  trajectory.getRobotTrajectoryMsg(output);
  return output;
}

}  // namespace rlc_planner
