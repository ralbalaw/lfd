#pragma once

#include <cstdint>
#include <string>

namespace rlc_executive
{

/**
 * @brief Execution result status values written to BT output ports.
 */
enum class ExecStatus : std::uint8_t
{
  kUnknown = 0,
  kSuccess = 1,
  kFailure = 2,
  kCanceled = 3,
  kTimeout = 4
};

/**
 * @brief Planning result status values written to BT output ports.
 */
enum class PlanStatus : std::uint8_t
{
  kUnknown = 0,
  kSuccess = 1,
  kFailure = 2,
  kTimeout = 3
};

/**
 * @brief Summary of a motion planning attempt for BT outputs.
 *
 * @details
 * This type intentionally excludes the planned trajectory to keep BehaviorTree
 * blackboard values small. Publish the trajectory separately.
 */
struct PlanSummary
{
  PlanStatus status = PlanStatus::kUnknown;

  int32_t moveit_error_code = 0;
  std::string message;

  double planning_time_sec = 0.0;
};

/**
 * @brief Result of a trajectory execution attempt for BT outputs.
 */
struct ExecResult
{
  ExecStatus status = ExecStatus::kUnknown;

  int32_t error_code = 0;
  std::string message;

  double execution_time_sec = 0.0;
};

}  // namespace rlc_executive
