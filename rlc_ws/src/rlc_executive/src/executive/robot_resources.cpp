#include "rlc_executive/executive/robot_resources.hpp"

#include <stdexcept>
#include <utility>

#include "rlc_utils/tesseract/tesseract_utils.hpp"

namespace rlc_executive
{

RobotResources RobotResources::create(rclcpp::Node::SharedPtr node, rclcpp::Logger logger,
                                      Options opt)
{
  if (!node)
  {
    throw std::invalid_argument("RobotResources: node is null");
  }

  const auto current_time = node->now();

  RobotResources resources(std::move(node), std::move(logger), std::move(opt));
  resources.start(current_time);

  return resources;
}

RobotResources::RobotResources(rclcpp::Node::SharedPtr node, rclcpp::Logger logger,
                               Options opt)
  : opt_(std::move(opt)), logger_(logger.get_child("RobotResources"))
{
  robot_model_loader::RobotModelLoader::Options model_options(opt_.robot_description);
  model_options.load_kinematics_solvers = false;

  robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(node, model_options);

  robot_model_ = robot_model_loader_->getModel();

  if (!robot_model_)
  {
    throw std::runtime_error("RobotResources: failed to load the robot model");
  }

  scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, robot_model_loader_, "motion_planning_context");

  if (!scene_monitor_->getRobotModel())
  {
    throw std::runtime_error("RobotResources: failed to create the planning "
                             "scene monitor");
  }

  scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
      opt_.move_group_namespace);

  environment_monitor_ = rlc_utils::tesseract_utils::makeMonitorInterface(
      node, logger_, opt_.monitor_namespace, opt_.monitor_wait_timeout, opt_.env_name);

  if (!environment_monitor_)
  {
    throw std::runtime_error("RobotResources: failed to create Tesseract monitor "
                             "interface");
  }
}

void RobotResources::start(const rclcpp::Time& current_time)
{
  scene_monitor_->startStateMonitor(opt_.joint_states_topic);
  scene_monitor_->startSceneMonitor(opt_.monitored_scene_topic);

  if (!scene_monitor_->requestPlanningSceneState(opt_.get_scene_service))
  {
    throw std::runtime_error("RobotResources: failed to obtain initial planning scene");
  }

  if (!scene_monitor_->getStateMonitor()->waitForCurrentState(current_time,
                                                              opt_.initial_state_timeout))
  {
    throw std::runtime_error("RobotResources: timed out waiting for initial robot state");
  }

  const auto current_state = scene_monitor_->getStateMonitor()->getCurrentState();
  if (!current_state)
  {
    throw std::runtime_error("RobotResources: failed to cache initial robot state");
  }

  reference_state_.emplace(*current_state);
}
}  // namespace rlc_executive
