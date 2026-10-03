#pragma once

/**
 * @file trajopt_planning_context.hpp
 * @brief MoveIt `planning_interface::PlanningContext` implementation for TrajOpt.
 */

#include <memory>
#include <moveit/planning_interface/planning_interface.hpp>
#include <tesseract_environment/fwd.h>

#include <string>

namespace rlc_planner
{

class TrajOptInterface;

/**
 * @brief Planning context that delegates planning to TrajOpt via TrajOptInterface.
 *
 * @details
 * The context is configured by MoveIt via setPlanningScene() and setMotionPlanRequest(),
 * then solve() produces a `MotionPlanResponse` or `MotionPlanDetailedResponse`.
 *
 * The stored Tesseract environment snapshot is modified during planning; the caller must
 * ensure the environment instance is not used concurrently by other threads.
 */
class TrajOptPlanningContext final : public planning_interface::PlanningContext
{
public:
  using EnvironmentPtr = std::shared_ptr<tesseract_environment::Environment>;
  using TrajOptInterfacePtr = std::shared_ptr<TrajOptInterface>;

  /**
   * @brief Constructs a planning context.
   * @param[in] name Context name (typically the planner algorithm id).
   * @param[in] group MoveIt group name.
   * @param[in] trajopt_interface TrajOpt interface used to perform planning; must not be null.
   * @param[in] env_snapshot Tesseract environment snapshot used during planning; must not be null.
   */
  TrajOptPlanningContext(const std::string& name, const std::string& group,
                         TrajOptInterfacePtr trajopt_interface,
                         EnvironmentPtr env_snapshot);

  ~TrajOptPlanningContext() override;

  /**
   * @brief Solves the configured request and populates a standard MoveIt response.
   * @param[out] res Response populated by this call.
   *
   * @note On failure, `res.error_code` is populated with the corresponding MoveIt error
   * code and source string.
   */
  void solve(planning_interface::MotionPlanResponse& res) override;

  /**
   * @brief Solves the configured request and populates a detailed MoveIt response.
   * @param[out] res Detailed response populated by this call.
   */
  void solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /**
   * @brief Requests termination of an active solve.
   * @return False (termination is not currently supported).
   */
  bool terminate() override;

  /**
   * @brief Clears any internal state for the context.
   *
   * @note This is currently a no-op.
   */
  void clear() override;

private:
  TrajOptInterfacePtr trajopt_interface_;  ///< TrajOpt interface used by solve().
  EnvironmentPtr env_snapshot_;            ///< Tesseract environment snapshot used by solve().
};

}  // namespace rlc_planner
