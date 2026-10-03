#pragma once

/**
 * @file ompl_solver.hpp
 * @brief Native OMPL joint-space planning wrapper with Tesseract-based collision checking.
 */

#include <tesseract_environment/environment.h>

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// OMPL (native)
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>

// Tesseract collision config
#include <tesseract_collision/core/types.h>

#include "rbt_types/math.hpp"

namespace rbt_planning
{

class OMPLSolver
{
public:
  using JointVec = rbt_types::JointVec;
  using PlannerFactoryFn =
      std::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr&)>;

  explicit OMPLSolver(std::shared_ptr<tesseract_environment::Environment> env = nullptr);

  void setEnvironment(std::shared_ptr<tesseract_environment::Environment> env);
  void setManipulatorGroup(const std::string& group);

  /**
   * @brief Sets the start joint state and updates the environment state.
   * @param[in] joint_names Joint names corresponding to `q_start`, size = `q_start.size()`.
   * @param[in] q_start Start joint positions [rad], size = `joint_names.size()`.
   * @throws std::runtime_error If the environment is not set.
   * @throws std::invalid_argument If `joint_names.size() != q_start.size()`.
   */
  void setStartState(const std::vector<std::string>& joint_names,
                     const JointVec& q_start);

  /**
   * @brief Sets the goal joint state.
   * @param[in] q_goal Goal joint positions [rad], size = `q_start.size()`.
   * @throws std::runtime_error If setStartState() has not been called.
   * @throws std::invalid_argument If `q_goal.size() != q_start.size()`.
   */
  void setGoalState(const JointVec& q_goal);

  void setPlanningTime(double seconds);
  void setSimplify(bool enabled);

  void setLongestValidSegmentFraction(double fraction);

  void setContactManagerConfig(const tesseract_collision::ContactManagerConfig& cfg);

  void setPlannerFactory(PlannerFactoryFn fn);

  /**
   * @brief Builds the OMPL problem, runs solve(), and returns the solved setup object.
   * @return Solved OMPL `SimpleSetup` instance. Use `getLastPlannerStatus()` /
   * `haveSolutionPath()` to inspect the result.
   */
  std::shared_ptr<ompl::geometric::SimpleSetup> solve();

private:
  void attachValidityChecker(const std::shared_ptr<ompl::geometric::SimpleSetup>& ss);
  std::shared_ptr<ompl::geometric::SimpleSetup> buildProblem();

  std::shared_ptr<tesseract_environment::Environment> env_;

  std::string manipulator_group_{ "manipulator" };

  std::vector<std::string> joint_names_;
  JointVec q_start_;
  JointVec q_goal_;

  double planning_time_{ 5.0 };
  bool simplify_{ false };
  double lvs_fraction_{ 0.01 };

  tesseract_collision::ContactManagerConfig contact_cfg_{};

  PlannerFactoryFn planner_factory_{ nullptr };
  ompl::base::OptimizationObjectivePtr objective_{ nullptr };
};

}  // namespace rbt_planning
