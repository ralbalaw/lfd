#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rlc_interfaces/action/collect_demonstration.hpp>

#include "rlc_frontend/collect_demonstration_parameters.hpp"
#include "rlc_frontend/frontend/frontend_config.hpp"
#include "rlc_frontend/frontend/teleoperation_controller.hpp"

namespace rlc_frontend
{

namespace server_detail
{
class JointDemonstrationRecorder;
}

/// @brief Action server that records frontend-driven joint demonstrations.
class CollectDemonstrationServer final : public rclcpp::Node
{
public:
  /**
   * @brief Constructs the demonstration collection server.
   * @param[in] options ROS node options forwarded to `rclcpp::Node`.
   * @throws std::invalid_argument If server or teleoperation config is invalid.
   */
  explicit CollectDemonstrationServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief Destroys the demonstration collection server.
  ~CollectDemonstrationServer() override;

private:
  using CollectDemonstration = rlc_interfaces::action::CollectDemonstration;
  using GoalHandleCollectDemonstration =
      rclcpp_action::ServerGoalHandle<CollectDemonstration>;

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID& uuid,
             const std::shared_ptr<const CollectDemonstration::Goal>& goal);
  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle);

  void execute(const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle);

  void publishFeedback(const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
                       const std::string& feedback_msg, double elapsed_sec,
                       std::size_t sample_count);

  void succeedGoal(const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
                   const moveit_msgs::msg::RobotTrajectory& demonstration,
                   double elapsed_sec, std::size_t sample_count);
  void abortGoal(const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
                 const std::string& error, double elapsed_sec, std::size_t sample_count);
  void cancelGoal(const std::shared_ptr<GoalHandleCollectDemonstration>& goal_handle,
                  double elapsed_sec, std::size_t sample_count);
  void requestTeleoperationStopped();

  std::shared_ptr<rlc_frontend::ParamListener> param_listener_;
  CollectDemonstrationConfig cfg_;
  TeleoperationController teleoperation_controller_;
  std::unique_ptr<server_detail::JointDemonstrationRecorder> recorder_;

  rclcpp_action::Server<CollectDemonstration>::SharedPtr action_server_;

  std::atomic_bool active_goal_{ false };
};

}  // namespace rlc_frontend
