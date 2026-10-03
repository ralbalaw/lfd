#pragma once

/**
 * @file exceptions.hpp
 * @brief Package-specific exception types.
 */

#include <cstdint>
#include <stdexcept>
#include <string>

namespace rlc_planner
{

/**
 * @brief Exception used to propagate a MoveIt error code with a message.
 *
 * @details
 * The stored error code is intended to be reported to MoveIt callers (typically one of
 * `moveit_msgs::msg::MoveItErrorCodes::*`).
 */
class PlanningError : public std::runtime_error
{
public:
  /**
   * @brief Constructs a PlanningError.
   * @param[in] code MoveIt error code value (see `moveit_msgs::msg::MoveItErrorCodes`).
   * @param[in] msg Human-readable error message.
   */
  PlanningError(int32_t code, const std::string& msg)
    : std::runtime_error(msg), code_(code)
  {
  }

  /**
   * @brief Returns the stored MoveIt error code value.
   * @return Error code value.
   */
  int32_t code() const noexcept { return code_; }

private:
  int32_t code_;  ///< MoveIt error code value.
};

}  // namespace rlc_planner
