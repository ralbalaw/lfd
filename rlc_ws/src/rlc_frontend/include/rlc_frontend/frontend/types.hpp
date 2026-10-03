#pragma once

#include <cstdint>
#include <string>

#include <moveit_msgs/msg/servo_status.hpp>

namespace rlc_frontend
{

/**
 * @brief Result of a MoveIt Servo pause or resume request.
 *
 * @details
 * - `paused`: Requested Servo paused state.
 * - `success`: Whether the service request completed and was accepted.
 * - `message`: Human-readable service or transport status.
 */
struct ServoPauseResult
{
  bool paused = true;
  bool success = false;
  std::string message;
};

/**
 * @brief Result of a MoveIt Servo command-type switch request.
 *
 * @details
 * - `command_type`: Requested MoveIt Servo command type.
 * - `success`: Whether the service request completed and was accepted.
 * - `message`: Human-readable service or transport status.
 */
struct ServoCommandTypeResult
{
  std::int8_t command_type = 0;
  bool success = false;
  std::string message;
};

/**
 * @brief Latest MoveIt Servo status with receive-time age.
 *
 * @details
 * - `status`: Last Servo status message.
 * - `age_sec`: Age [s] of the cached status sample at read time.
 */
struct ServoStatusSnapshot
{
  moveit_msgs::msg::ServoStatus status;
  double age_sec = 0.0;
};

/**
 * @brief Operator request emitted by the active demonstration frontend.
 */
enum class DemonstrationRequest
{
  kNone,
  kFinish,
  kAbort
};

}  // namespace rlc_frontend
