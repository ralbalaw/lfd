#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <tesseract_environment/environment.h>

#include "rbt_types/joint_endpoints.hpp"
#include "rbt_types/joint_path.hpp"
#include "rbt_types/math.hpp"
#include "rbt_types/objective_term.hpp"

namespace rbt_planning
{

/**
 * @brief Plans optimized joint paths with optional OMPL seeding and TrajOpt refinement.
 *
 * @details
 * Joint vectors use the ordering returned by the configured Tesseract manipulator group.
 * The returned paths contain joint positions only; no timing information is attached.
 */
class TrajOptPlanner
{
public:
  using JointVec = rbt_types::JointVec;
  using RewardTerms = rbt_types::RewardTerms;
  using JointPath = rbt_types::JointPath;
  using JointPaths = rbt_types::JointPaths;
  using JointEndpointsSet = rbt_types::JointEndpointsSet;
  using JointEndpoints = rbt_types::JointEndpoints;

  /**
   * @brief Planner configuration.
   *
   * @details
   * - `ompl_planning_time`: OMPL planning time limit [s].
   * - `ompl_simplify`: Whether OMPL simplifies the seed path.
   * - `ompl_longest_valid_segment_fraction`: OMPL validity-check resolution fraction.
   * - `trajopt_num_steps`: Number of TrajOpt path waypoints.
   * - `use_ompl_seed`: Whether to seed TrajOpt from OMPL before falling back to a
   *   linear seed.
   */
  struct Options
  {
    double ompl_planning_time{ 5.0 };
    bool ompl_simplify{ false };
    double ompl_longest_valid_segment_fraction{ 0.01 };

    int trajopt_num_steps{ 20 };
    bool use_ompl_seed{ true };
  };

  /**
   * @brief Constructs a planner with default options.
   * @param[in] env Tesseract environment; must not be nullptr and must contain
   * `planning_group`.
   * @param[in] planning_group Manipulator group name used for planning.
   * @throws std::invalid_argument If `env` is nullptr.
   * @throws std::runtime_error If `planning_group` has no joints in `env`.
   */
  explicit TrajOptPlanner(std::shared_ptr<tesseract_environment::Environment> env,
                          const std::string& planning_group);

  /**
   * @brief Constructs a planner with explicit options.
   * @param[in] env Tesseract environment; must not be nullptr and must contain
   * `planning_group`.
   * @param[in] planning_group Manipulator group name used for planning.
   * @param[in] opt Planner options.
   * @throws std::invalid_argument If `env` is nullptr.
   * @throws std::runtime_error If `planning_group` has no joints in `env`.
   */
  explicit TrajOptPlanner(std::shared_ptr<tesseract_environment::Environment> env,
                          const std::string& planning_group, const Options& opt);

  /**
   * @brief Plans one joint path for each start/goal pair.
   * @param[in] endpoints_set Start and goal joint positions [rad], length = num_paths;
   * each endpoint vector has size = dof.
   * @param[in] rewards Reward terms evaluated over each planned path.
   * @return Joint paths, length = `endpoints_set.size()`.
   * @throws std::runtime_error If planning fails or the environment/group is invalid.
   */
  JointPaths plan(const JointEndpointsSet& endpoints_set,
                  const RewardTerms& rewards) const;

  /**
   * @brief Plans a joint-space path between start and goal states.
   * @param[in] endpoints Start and goal joint positions [rad], size = dof.
   * @param[in] rewards Reward terms evaluated over the planned path.
   * @return Joint-space path [rad], shape = (`Options::trajopt_num_steps`, dof).
   * @throws std::runtime_error If planning fails or the environment/group is invalid.
   */
  JointPath plan(const JointEndpoints& endpoints, const RewardTerms& rewards) const;

private:
  std::shared_ptr<tesseract_environment::Environment> env_;
  std::string planning_group_;
  std::vector<std::string> joint_names_;
  Options opt_{};
};

}  // namespace rbt_planning
