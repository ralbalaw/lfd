#pragma once

#include <string>

namespace rlc_executive
{

/**
 * @brief Planning parameters loaded from planning-config YAML.
 *
 * @details
 * MoveIt planning request fields:
 * - `pipeline_id`: Planning pipeline id (for example, `"ompl"`).
 * - `planner_id`: Planner id (for example, `"RRTConnectkConfigDefault"`).
 * - `allowed_planning_time_sec`: Allowed planning time [s].
 * - `num_planning_attempts`: Number of planning attempts.
 * - `max_velocity_scaling_factor`: Velocity scaling factor (0, 1].
 * - `max_acceleration_scaling_factor`: Acceleration scaling factor (0, 1].
 */
struct PlanningConfig
{
  std::string pipeline_id;
  std::string planner_id;

  double allowed_planning_time_sec{ 5.0 };
  int num_planning_attempts{ 1 };
  double max_velocity_scaling_factor{ 1.0 };
  double max_acceleration_scaling_factor{ 1.0 };
};

}  // namespace rlc_executive
