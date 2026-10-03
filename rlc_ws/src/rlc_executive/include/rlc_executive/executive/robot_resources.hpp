#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/current_state_monitor.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tf2_ros/buffer.h>

namespace rlc_executive
{

/**
 * @brief Owns shared MoveIt and Tesseract robot resources used by BT nodes.
 *
 * @details
 * `RobotResources` is move-only. Create it through create() so model loading,
 * monitor startup, planning-scene sync, and startup reference-state caching happen
 * as one operation.
 */
class RobotResources
{
public:
  /**
   * @brief Runtime resource settings used to construct robot resources.
   *
   * @details
   * - `robot_description`: ROS parameter containing URDF text.
   * - `joint_states_topic`: Joint state topic used by MoveIt state monitoring.
   * - `monitored_scene_topic`: Planning-scene diff topic consumed by the local
   *   monitor.
   * - `get_scene_service`: MoveIt service used to request the initial full scene.
   * - `move_group_namespace`: Namespace used by PlanningSceneInterface.
   * - `monitor_namespace`: Tesseract monitor namespace.
   * - `env_name`: Tesseract environment name.
   * - `monitor_wait_timeout`: Maximum wait time [ms] for the Tesseract monitor.
   * - `initial_state_timeout`: Maximum wait time [s] for the first robot state.
   */
  struct Options
  {
    std::string robot_description{ "robot_description" };
    std::string joint_states_topic{ "/joint_states" };
    std::string monitored_scene_topic{ "monitored_planning_scene" };
    std::string get_scene_service{ "/get_planning_scene" };
    std::string move_group_namespace;
    std::string monitor_namespace{ "tesseract_monitor" };
    std::string env_name{ "default" };

    std::chrono::milliseconds monitor_wait_timeout{ 3000 };
    double initial_state_timeout{ 5.0 };
  };

  RobotResources(rclcpp::Node::SharedPtr node, rclcpp::Logger logger, Options options);

  /**
   * @brief Creates and starts all robot resources.
   * @param[in] node ROS node used by MoveIt and Tesseract clients; must not be
   * nullptr and must outlive this object.
   * @param[in] logger Logger used during startup.
   * @param[in] config Runtime resource settings.
   * @return Fully started robot resources.
   * @throws std::runtime_error If model loading, monitor creation, scene sync, or
   * initial state acquisition fails.
   */
  static RobotResources create(rclcpp::Node::SharedPtr node, rclcpp::Logger logger,
                               Options options);

  /**
   * @brief Returns the loaded MoveIt robot model.
   * @return Shared robot model pointer owned by the MoveIt loader; valid while this
   * object lives.
   */
  const moveit::core::RobotModelConstPtr& model() const noexcept
  { return robot_model_; }

  /**
   * @brief Returns the MoveIt robot model loader.
   * @return Shared loader pointer; valid while this object lives.
   */
  robot_model_loader::RobotModelLoader* robotModelLoader() const noexcept
  { return robot_model_loader_.get(); }

  /**
   * @brief Returns the synchronized local planning-scene monitor.
   * @return Retained pointer; valid while this object lives.
   */
  planning_scene_monitor::PlanningSceneMonitor* sceneMonitor() const noexcept
  { return scene_monitor_.get(); }

  /**
   * @brief Returns the remote planning-scene interface.
   * @return Retained pointer; valid while this object lives.
   */
  moveit::planning_interface::PlanningSceneInterface* sceneInterface() const noexcept
  { return scene_interface_.get(); }

  /**
   * @brief Returns the MoveIt current-state monitor owned by the scene monitor.
   * @return Retained pointer; valid while this object lives.
   */
  planning_scene_monitor::CurrentStateMonitor* currentStateMonitor() const noexcept
  { return scene_monitor_->getStateMonitor().get(); }

  /**
   * @brief Returns the TF buffer owned by the planning-scene monitor.
   * @return Retained pointer; valid while this object lives.
   */
  tf2_ros::Buffer* tfBuffer() const noexcept
  { return scene_monitor_->getTFClient().get(); }

  /**
   * @brief Returns the Tesseract environment monitor interface.
   * @return Retained pointer; valid while this object lives.
   */
  tesseract_monitoring::ROSEnvironmentMonitorInterface* environmentMonitor() noexcept
  { return &(*environment_monitor_); }

  /**
   * @brief Returns the startup reference robot state.
   * @return Retained pointer; valid while this object lives.
   */
  const moveit::core::RobotState* referenceState() const noexcept
  { return &(*reference_state_); }

private:
  void start(const rclcpp::Time& current_time);

  Options opt_;
  rclcpp::Logger logger_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_interface_;

  std::optional<tesseract_monitoring::ROSEnvironmentMonitorInterface> environment_monitor_;
  std::optional<moveit::core::RobotState> reference_state_;
};
}  // namespace rlc_executive
