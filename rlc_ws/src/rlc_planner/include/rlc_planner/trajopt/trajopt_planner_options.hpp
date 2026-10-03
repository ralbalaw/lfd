#pragma once

/**
 * @file trajopt_planner_options.hpp
 * @brief Parameter-backed option structs for the TrajOpt MoveIt planner plugin.
 */

#include <chrono>
#include <string>

namespace rlc_planner
{

/**
 * @brief Aggregated options for the `rlc_trajopt` MoveIt planner plugin.
 *
 * @details
 * - `monitor` configures connectivity to the Tesseract environment monitor.
 * - `trajopt` configures TrajOpt-specific solve settings and caching behavior.
 */
struct TrajOptPlannerOptions
{
  /**
   * @brief Options for connecting to a Tesseract environment monitoring node.
   *
   * @details
   * - `monitor_namespace` selects which monitor namespace to connect to.
   * - `env_name` selects which environment name to snapshot (must match the monitor's
   *   environment name).
   * - `wait_timeout` controls how long initialize() waits for the monitor namespace to
   *   become available.
   */
  struct MonitorOptions
  {
    std::string monitor_namespace{ "tesseract_monitor" };
    std::string env_name{ "default" };
    std::chrono::milliseconds wait_timeout{ std::chrono::milliseconds{ 3000 } };
  };

  /**
   * @brief TrajOpt interface options that are fixed at initialization time.
   *
   * @details
   * - `default_num_steps`: Default number of waypoints in the optimization (must be >= 2).
   * - `use_moveit_group_as_tesseract_manipulator`: If true, the MoveIt `req.group_name`
   *   is used as the Tesseract manipulator group.
   * - `fixed_tesseract_manipulator_group`: Used when
   *   `use_moveit_group_as_tesseract_manipulator` is false (falls back to `"manipulator"`
   *   if empty).
   * - `enable_seed_cache`: Enables caching of seed hints across calls.
   * - `enable_last_solution_cache`: Enables caching of last solutions across calls.
   */
  struct TrajOptOptions
  {
    int default_num_steps{ 20 };

    bool use_moveit_group_as_tesseract_manipulator{ true };
    std::string fixed_tesseract_manipulator_group{ "manipulator" };

    bool enable_seed_cache{ false };
    bool enable_last_solution_cache{ false };
  };

  MonitorOptions monitor{};
  TrajOptOptions trajopt{};
};

}  // namespace rlc_planner
