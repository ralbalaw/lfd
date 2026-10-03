#pragma once

/**
 * @file trajopt_planner_manager.hpp
 * @brief MoveIt `planning_interface::PlannerManager` plugin for TrajOpt.
 */

#include <rlc_planner/trajopt/trajopt_planner_options.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <rclcpp/node.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tesseract_monitoring
{
class ROSEnvironmentMonitorInterface;
}  // namespace tesseract_monitoring

namespace rlc_planner
{

class TrajOptInterface;

/**
 * @brief MoveIt planner manager that constructs TrajOpt planning contexts.
 *
 * @details
 * This plugin exposes a single planning algorithm id: `"rlc_trajopt"`.
 *
 * The manager connects to a Tesseract monitoring node and snapshots a Tesseract
 * environment for each planning context.
 */
class TrajOptPlannerManager final : public planning_interface::PlannerManager
{
public:
  using planning_interface::PlannerManager::getPlanningContext;

  /**
   * @brief Constructs an uninitialized planner manager.
   *
   * @note The instance must be initialized via initialize() before planning requests can
   * be serviced.
   */
  TrajOptPlannerManager();

  ~TrajOptPlannerManager() override = default;

  /**
   * @brief Initializes this planner plugin.
   * @param[in] model Robot model provided by MoveIt.
   * @param[in] node Node used for parameters, services, and logging.
   * @param[in] parameter_namespace Namespace prefix for parameters declared by this plugin.
   * @return True on success, false on failure.
   *
   * @details
   * Parameters are declared on `node` under `parameter_namespace` (if non-empty), then
   * used to configure the Tesseract monitor connectivity and TrajOpt interface.
   */
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  /**
   * @brief Returns the human-readable planner description.
   * @return Planner description string.
   */
  std::string getDescription() const override;

  /**
   * @brief Populates the list of planner algorithm ids supported by this plugin.
   * @param[out] algs Output list of algorithm ids.
   */
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  /**
   * @brief Creates and configures a planning context for the given request.
   * @param[in] planning_scene Planning scene used by the context.
   * @param[in] req Motion plan request.
   * @param[out] error_code MoveIt error code populated on return.
   * @return A planning context on success, or nullptr on failure.
   *
   * @note On failure, `error_code` is populated with a non-success code and a message.
   */
  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  /**
   * @brief Checks whether this plugin can service the given request.
   * @param[in] req Motion plan request.
   * @return True if the request is supported by this planner plugin.
   *
   * @details
   * This checks group existence and planner id match. It does not reject
   * `req.trajectory_constraints` (interpreted as a joint seed trajectory).
   */
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

private:
  /**
   * @brief Declares and loads parameters under the given prefix.
   * @param[in] pfx Parameter name prefix (typically `"<ns>."` or empty).
   * @return Loaded options.
   */
  TrajOptPlannerOptions declareAndLoadOptions(const std::string& pfx);

  /**
   * @brief Initializes the Tesseract environment monitor interface.
   * @param[in] node Node used for ROS interfaces.
   * @param[in] opt Monitor options.
   * @return True if the monitor interface was constructed and connectivity was
   * established.
   */
  bool initializeTesseractMonitor(const rclcpp::Node::SharedPtr& node,
                                  const TrajOptPlannerOptions::MonitorOptions& opt);

  /**
   * @brief Validates that the request can be serviced in the current plugin state.
   * @param[in] planning_scene Planning scene.
   * @param[in] req Motion plan request.
   * @throws rlc_planner::PlanningError If any required state is missing or invalid.
   */
  void validateRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const planning_interface::MotionPlanRequest& req) const;

  moveit::core::RobotModelConstPtr model_;  ///< Robot model provided by MoveIt.
  rclcpp::Node::SharedPtr node_;            ///< Node used for parameters/logging.
  std::string parameter_namespace_;         ///< Parameter namespace prefix for this plugin.
  TrajOptPlannerOptions options_{};         ///< Loaded options used for initialization.

  rclcpp::Node::SharedPtr tesseract_monitor_client_node_;  ///< Dedicated node for Tesseract monitor service calls.

  std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitorInterface>
      monitor_interface_;  ///< Monitor interface used to snapshot Tesseract environments.
  std::shared_ptr<TrajOptInterface> trajopt_interface_;  ///< Shared TrajOpt interface.
};

}  // namespace rlc_planner
