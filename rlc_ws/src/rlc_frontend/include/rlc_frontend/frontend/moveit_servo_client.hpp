#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>

#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "rlc_frontend/frontend/frontend_config.hpp"
#include "rlc_frontend/frontend/types.hpp"

namespace rlc_frontend
{

/**
 * @brief Wraps MoveIt Servo control services and the Servo status topic.
 */
class MoveItServoClient final
{
public:
  using PauseSrv = std_srvs::srv::SetBool;
  using CmdTypeSrv = moveit_msgs::srv::ServoCommandType;

  /// @brief MoveIt Servo command input type selected through the command-type service.
  enum class CommandType : std::int8_t
  {
    kJointJog = CmdTypeSrv::Request::JOINT_JOG,
    kTwist = CmdTypeSrv::Request::TWIST,
    kPose = CmdTypeSrv::Request::POSE,
  };

  /**
   * @brief Creates clients for the MoveIt Servo control services and status topic.
   * @param[in] node ROS node used to create the clients, subscription, and timestamps.
   * @param[in] cfg Servo configuration with service and topic names.
   * @throws std::invalid_argument If any required service or topic name is empty.
   */
  MoveItServoClient(rclcpp::Node* node, const ServoConfig* cfg);

  /**
   * @brief Pauses or resumes MoveIt Servo through the configured service.
   * @param[in] paused `true` to pause Servo, `false` to resume Servo.
   * @param[in] timeout Maximum wait time [ms] for service discovery and response.
   * @return Service result and requested pause state.
   */
  ServoPauseResult
  setPaused(bool paused,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(300));

  /**
   * @brief Switches the MoveIt Servo command input type.
   * @param[in] type Command input type to request.
   * @param[in] timeout Maximum wait time [ms] for service discovery and response.
   * @return Service result and requested command type.
   */
  ServoCommandTypeResult
  setCommandType(CommandType type,
                 std::chrono::milliseconds timeout = std::chrono::milliseconds(300));

  /**
   * @brief Returns the latest cached MoveIt Servo status.
   * @return Latest status snapshot, or `std::nullopt` if no status has been received.
   */
  std::optional<ServoStatusSnapshot> status() const;

private:
  void onStatus(const moveit_msgs::msg::ServoStatus::ConstSharedPtr& msg);

  rclcpp::Node* node_;
  const ServoConfig* cfg_;

  rclcpp::Client<PauseSrv>::SharedPtr pause_client_;
  rclcpp::Client<CmdTypeSrv>::SharedPtr cmd_type_client_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_sub_;

  mutable std::mutex status_mtx_;
  std::optional<moveit_msgs::msg::ServoStatus> last_status_;
  rclcpp::Time last_status_stamp_;
};

}  // namespace rlc_frontend
