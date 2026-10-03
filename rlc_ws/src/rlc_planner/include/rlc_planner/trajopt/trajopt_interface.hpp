#pragma once

/**
 * @file trajopt_interface.hpp
 * @brief MoveIt-to-TrajOpt interface for joint-space motion planning requests.
 */

#include <rbt_planning/trajopt_solver.hpp>
#include <rbt_types/objective_term.hpp>
#include <rlc_planner/trajopt/trajopt_planner_options.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_state.hpp>

#include <rclcpp/node.hpp>

#include <tesseract_environment/environment.h>

#include <trajopt/utils.hpp>

#include <Eigen/Core>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rlc_planner
{

/**
 * @brief Translates a MoveIt planning request into a TrajOpt solve and converts the
 * result back to MoveIt.
 *
 * @details
 * This interface extracts a joint-space start state from the planning scene and a
 * joint-only goal from the request, constructs a TrajOpt problem via
 * `rbt_planning::TrajOptSolver`, and returns a `robot_trajectory::RobotTrajectory`.
 *
 * Current limitations (MVP):
 * - Only 1-DOF active joints are supported in the MoveIt group.
 * - Only `goal_constraints[*].joint_constraints` goals are supported (no pose/visibility
 * constraints).
 * - If `req.trajectory_constraints.constraints` is non-empty, it is interpreted as a
 * joint-space seed trajectory (planner chaining). Each waypoint must provide exactly one
 * `JointConstraint` for each active MoveIt joint, and must not include non-joint
 * constraints.
 * - The MoveIt active joint list must exactly match the configured Tesseract manipulator
 * group joint list (same names and order).
 *
 * Feature costs can be injected via setFeatureCosts() and are applied on every solve.
 * Options are fixed at construction time and returned by options().
 *
 * @par Thread safety
 * Thread-safe with respect to internal configuration and caches (synchronized with
 * mutexes). The provided Tesseract environment snapshot is modified during planning;
 * callers must ensure the same environment instance is not used concurrently by other
 * threads.
 */
class TrajOptInterface final
{
public:
  using JointVec = rbt_types::JointVec;

  /**
   * @brief Planning result produced by plan().
   *
   * @details
   * - `trajectory` is a MoveIt `RobotTrajectory` for the requested MoveIt group
   * containing joint positions [rad].
   * - `start_state_msg` is the start state actually used after applying `req.start_state`
   * to the scene.
   *
   * @note The returned trajectory currently uses `waypoint_dt = 0.0` for all waypoints
   * (no timing).
   */
  struct PlanResult
  {
    robot_trajectory::RobotTrajectoryPtr trajectory;  ///< Planned joint-space trajectory.
    moveit_msgs::msg::RobotState
        start_state_msg;  ///< Start state message used for planning.
  };

  /**
   * @brief Constructs a TrajOpt interface with fixed, request-invariant options.
   * @param[in] opt TrajOpt planner options (moved into this instance).
   * @note Invalid option values may be clamped to safe defaults (e.g., `default_num_steps >= 2`).
   */
  explicit TrajOptInterface(TrajOptPlannerOptions::TrajOptOptions opt);

  /**
   * @brief Plans a joint-space trajectory using TrajOpt.
   * @param[in] planning_scene Planning scene used to resolve the robot model and start
   * state.
   * @param[in] req MoveIt motion plan request (uses `group_name`, `start_state`, and
   * joint goal constraints).
   * @param[in,out] env_snapshot Tesseract environment snapshot used for collision
   * checking and kinematics; must not be nullptr. The environment state is updated to the
   * start state during planning.
   * @return Planning result including a MoveIt trajectory and the used start state
   * message.
   * @throws rlc_planner::PlanningError If the request is invalid/unsupported, inputs
   * violate bounds, the seed trajectory is invalid, the joint order does not match
   * between MoveIt and Tesseract, the TrajOpt solve fails, or the resulting trajectory
   * fails MoveIt validity checks.
   * @pre `req.group_name` exists in the scene robot model.
   * @pre `env_snapshot != nullptr`.
   * @pre MoveIt active joints for `req.group_name` exactly match the Tesseract joint list
   * for the selected manipulator group (same names and order).
   */
  PlanResult
  plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
       const planning_interface::MotionPlanRequest& req,
       const std::shared_ptr<tesseract_environment::Environment>& env_snapshot) const;

  /**
   * @brief Replaces the set of user-defined TrajOpt reward terms applied on each solve.
   * @param[in] rewards Reward terms to apply (stored internally by value).
   * @note Thread-safe. Subsequent plan() calls use the updated rewards.
   */
  void setRewards(const rbt_types::RewardTerms& rewards);

  /**
   * @brief Returns the request-invariant TrajOpt options.
   * @return Planner options (reference valid for the lifetime of this interface).
   */
  const TrajOptPlannerOptions::TrajOptOptions& options() const
  { return options_; }

private:
  /**
   * @brief Cached seed hint entry.
   *
   * @details
   * - `joint_names`: Joint names in MoveIt active joint order, size = dof.
   * - `seed`: Seed trajectory, shape = (num_waypoints, dof) matching `joint_names` order.
   */
  struct SeedCacheEntry
  {
    std::vector<std::string> joint_names;
    trajopt::TrajArray seed;
  };

  /**
   * @brief Cached last-solution entry.
   *
   * @details
   * - `joint_names`: Joint names in MoveIt active joint order, size = dof.
   * - `traj`: Last solution trajectory, shape = (num_waypoints, dof) matching
   * `joint_names` order.
   */
  struct SolutionCacheEntry
  {
    std::vector<std::string> joint_names;
    trajopt::TrajArray traj;
  };

  /**
   * @brief Start-state data extracted from a MoveIt planning request.
   *
   * @details
   * Joint ordering for all vectors in this struct matches MoveIt active joint order for
   * the requested group.
   * - `joint_names`: Active joint names, size = dof.
   * - `variable_names`: Corresponding variable names (1-DOF joints only), size = dof.
   * - `q_start`: Start joint positions [rad], size = dof.
   * - `start_state`: Start MoveIt RobotState (full robot state).
   */
  struct StartData
  {
    explicit StartData(const moveit::core::RobotModelConstPtr& robot_model)
      : start_state(robot_model)
    {
    }

    std::vector<std::string> joint_names;
    std::vector<std::string> variable_names;
    JointVec q_start;
    moveit::core::RobotState start_state;
  };

  /**
   * @brief Joint-only goal data extracted from a MoveIt planning request.
   *
   * @details
   * - `goal_index`: Index of the selected goal constraint in `req.goal_constraints`.
   * - `goal_raw`: Raw MoveIt constraint message used to derive the goal.
   * - `q_goal`: Goal joint positions [rad], size = dof (StartData.joint_names order).
   * - `goal_state`: Goal MoveIt RobotState (full robot state).
   */
  struct GoalData
  {
    explicit GoalData(const moveit::core::RobotModelConstPtr& robot_model)
      : goal_state(robot_model)
    {
    }

    std::size_t goal_index{ 0 };
    moveit_msgs::msg::Constraints goal_raw;
    JointVec q_goal;  ///< Goal joint positions [rad] (StartData.joint_names order).
    moveit::core::RobotState goal_state;
  };

private:
  /**
   * @brief Extracts and validates the joint-space start state for the request.
   * @param[in] scene Planning scene providing the current state.
   * @param[in] req MoveIt motion plan request.
   * @param[in] jmg Joint model group for `req.group_name`.
   * @return Extracted start-state data.
   * @throws rlc_planner::PlanningError If the start state is invalid or violates bounds.
   */
  StartData extractStart(const planning_scene::PlanningSceneConstPtr& scene,
                         const planning_interface::MotionPlanRequest& req,
                         const moveit::core::JointModelGroup& jmg) const;

  /**
   * @brief Extracts and validates a joint-only goal constraint for the request.
   * @param[in] scene Planning scene providing the current state.
   * @param[in] req MoveIt motion plan request.
   * @param[in] jmg Joint model group for `req.group_name`.
   * @param[in] start Extracted start-state data used as a baseline for unspecified joints.
   * @return Extracted goal-state data.
   * @throws rlc_planner::PlanningError If the goal constraints are invalid or unsupported.
   */
  GoalData extractGoal(const planning_scene::PlanningSceneConstPtr& scene,
                       const planning_interface::MotionPlanRequest& req,
                       const moveit::core::JointModelGroup& jmg,
                       const StartData& start) const;

  /**
   * @brief Extracts an optional joint seed trajectory from `req.trajectory_constraints`.
   * @param[in] req MoveIt motion plan request.
   * @param[in] jmg Joint model group for `req.group_name`.
   * @return Seed trajectory in MoveIt active joint order, or nullptr if not provided.
   * @throws rlc_planner::PlanningError If the provided seed is malformed or unsupported.
   */
  std::shared_ptr<const trajopt::TrajArray>
  extractTrajectorySeed(const planning_interface::MotionPlanRequest& req,
                        const moveit::core::JointModelGroup& jmg) const;

  /**
   * @brief Runs the TrajOpt solve for the given start/goal and optional seed.
   * @param[in] req MoveIt motion plan request.
   * @param[in,out] env_snapshot Tesseract environment snapshot used for planning.
   * @param[in] start Extracted start-state data.
   * @param[in] goal Extracted goal-state data.
   * @param[in,out] seed Optional seed trajectory (may be nullptr).
   * @return Joint trajectory, shape = (num_steps, dof) in MoveIt active joint order.
   * @throws rlc_planner::PlanningError If the solve fails or joint ordering is incompatible.
   */
  trajopt::TrajArray
  solveTrajOpt(const planning_interface::MotionPlanRequest& req,
               const std::shared_ptr<tesseract_environment::Environment>& env_snapshot,
               const StartData& start, const GoalData& goal,
               std::shared_ptr<const trajopt::TrajArray>& seed) const;

  /**
   * @brief Converts a TrajOpt joint trajectory to a MoveIt RobotTrajectory.
   * @param[in] trajopt_traj Joint trajectory, shape = (num_waypoints, dof) in MoveIt joint order.
   * @param[in] robot_model Robot model for the trajectory.
   * @param[in] group_name MoveIt group name.
   * @param[in] start Extracted start-state data.
   * @return MoveIt RobotTrajectory (no timing).
   */
  static robot_trajectory::RobotTrajectoryPtr
  toMoveItTrajectory(const trajopt::TrajArray& trajopt_traj,
                     const moveit::core::RobotModelConstPtr& robot_model,
                     const std::string& group_name, const StartData& start);

  /**
   * @brief Resolves the Tesseract manipulator group name used for the solve.
   * @param[in] req MoveIt motion plan request.
   * @return Tesseract manipulator group name.
   */
  std::string
  resolveTesseractManipulatorGroup(const planning_interface::MotionPlanRequest& req) const;

  /**
   * @brief Returns a cached seed hint, if enabled and present.
   * @param[in] key Cache key.
   * @return Cached seed hint, or std::nullopt.
   */
  std::optional<SeedCacheEntry> getSeedHint(const std::string& key) const;

  /**
   * @brief Updates the seed hint cache entry for a key, if enabled.
   * @param[in] key Cache key.
   * @param[in] entry Cache entry to store.
   */
  void updateSeedHint(const std::string& key, SeedCacheEntry entry) const;

  /**
   * @brief Returns a cached last solution, if enabled and present.
   * @param[in] key Cache key.
   * @return Cached solution, or std::nullopt.
   */
  std::optional<SolutionCacheEntry> getLastSolution(const std::string& key) const;

  /**
   * @brief Updates the last-solution cache entry for a key, if enabled.
   * @param[in] key Cache key.
   * @param[in] entry Cache entry to store.
   */
  void updateLastSolution(const std::string& key, SolutionCacheEntry entry) const;

private:
  TrajOptPlannerOptions::TrajOptOptions options_;  ///< Request-invariant TrajOpt options.

  mutable std::mutex config_mutex_;          ///< Protects feature cost configuration.
  rbt_types::CostTerms costs_;  ///< Costs applied on each solve.

  mutable std::mutex cache_mutex_;  ///< Protects caches below.
  mutable std::unordered_map<std::string, SeedCacheEntry>
      seed_cache_;  ///< Optional seed hint cache.
  mutable std::unordered_map<std::string, SolutionCacheEntry>
      solution_cache_;  ///< Optional last-solution cache.
};

}  // namespace rlc_planner
