#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/node.hpp>

#include "rlc_frontend/frontend/frontend_config.hpp"
#include "rlc_frontend/frontend/moveit_servo_client.hpp"
#include "rlc_frontend/frontend/teleoperation_frontend.hpp"
#include "rlc_frontend/frontend/teleoperation_session.hpp"
#include "rlc_frontend/frontend/types.hpp"

namespace rlc_frontend
{

/**
 * @brief Owns the Servo teleoperation stack used by the demonstration server.
 *
 * @details
 * This controller owns the MoveIt Servo client, the teleoperation request session,
 * and the selected operator-input frontend. Callers should interact with teleoperation
 * through this class instead of wiring lower-level teleop objects separately.
 */
class TeleoperationController final
{
public:
  /**
   * @brief Constructs the complete Servo teleoperation stack.
   * @param[in] node ROS node used to create teleoperation ROS interfaces.
   * @param[in] teleop_cfg Top-level teleoperation configuration, including the selected
   * Servo client and frontend settings.
   * @throws std::invalid_argument If any owned dependency rejects the provided configuration.
   */
  TeleoperationController(rclcpp::Node* node, const TeleoperationConfig& teleop_cfg);

  /**
   * @brief Activates or deactivates Servo teleoperation as one operation.
   * @param[in] active `true` to select Twist commands, unpause Servo, and enable the
   * active frontend; `false` to disable the active frontend and pause Servo.
   * @param[in] timeout Maximum wait time [ms] used by each Servo service call.
   * @return Result of the Servo service request that determines whether teleoperation
   * became active.
   */
  ServoPauseResult
  setActive(bool active,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(300));

  /**
   * @brief Returns whether teleoperation is currently active.
   * @return `true` after Servo has been resumed and the frontend has been enabled.
   */
  bool isActive() const noexcept
  {
    return active_.load(std::memory_order_relaxed);
  }

  /**
   * @brief Returns and clears the currently latched operator request.
   * @return Pending operator request, or `DemonstrationRequest::kNone` if no request is pending.
   */
  DemonstrationRequest takeRequest();

  /**
   * @brief Returns the currently latched operator request without clearing it.
   * @return Pending operator request, or `DemonstrationRequest::kNone` if no request is pending.
   */
  DemonstrationRequest peekRequest() const;

  /**
   * @brief Returns the most recent Servo status sample, if one has been received.
   * @return Latest Servo status snapshot, or `std::nullopt` if no status has been observed.
   */
  std::optional<ServoStatusSnapshot> status() const;

private:
  TeleoperationConfig cfg_;
  MoveItServoClient servo_client_;
  TeleoperationSession session_;
  std::unique_ptr<TeleoperationFrontend> frontend_;
  std::atomic_bool active_{ false };
};

}  // namespace rlc_frontend
