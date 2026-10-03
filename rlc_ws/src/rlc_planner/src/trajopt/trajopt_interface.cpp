#include "rlc_planner/trajopt/trajopt_interface.hpp"

#include "rlc_planner/exceptions.hpp"

#include <moveit/robot_state/conversions.hpp>
#include <moveit/utils/logger.hpp>

#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <rbt_types/objective_term.hpp>
#include <sstream>

#include <unordered_map>
#include <vector>

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace rlc_planner
{

namespace
{

rclcpp::Logger getLogger()
{ return moveit::getLogger("rlc_planner.trajopt_interface"); }

std::string formatJointNames(const std::vector<std::string>& joint_names)
{
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    if (i != 0)
    {
      oss << ", ";
    }
    oss << joint_names[i];
  }
  oss << "]";
  return oss.str();
}

trajopt::TrajArray resampleByArcLength(const trajopt::TrajArray& traj_ref,
                                       std::size_t num_steps)
{
  if (num_steps == 0)
  {
    throw std::invalid_argument("resampleByArcLength: num_steps must be >= 1");
  }

  const int num_waypoints = static_cast<int>(traj_ref.rows());
  const int dof = static_cast<int>(traj_ref.cols());

  if (num_waypoints == 0 || dof == 0)
  {
    throw std::invalid_argument("resampleByArcLength: empty input");
  }

  trajopt::TrajArray traj_resampled(static_cast<int>(num_steps), dof);

  if (num_steps == 1)
  {
    traj_resampled.row(0) = traj_ref.row(0);
    return traj_resampled;
  }

  if (num_waypoints == 1)
  {
    throw std::invalid_argument("resampleByArcLength: cannot resample from one waypoint");
  }

  std::vector<double> s(static_cast<std::size_t>(num_waypoints), 0.0);
  for (int i = 1; i < num_waypoints; ++i)
  {
    s[static_cast<std::size_t>(i)] = s[static_cast<std::size_t>(i - 1)] +
                                     (traj_ref.row(i) - traj_ref.row(i - 1)).norm();
  }

  const double s_end = s.back();

  // Degenerate (all points same): just repeat
  if (!(s_end > 1e-12))
  {
    for (std::size_t k = 0; k < num_steps; ++k)
    {
      traj_resampled.row(static_cast<int>(k)) = traj_ref.row(0);
    }
    return traj_resampled;
  }

  for (std::size_t k = 0; k < num_steps; ++k)
  {
    const double u = static_cast<double>(k) / static_cast<double>(num_steps - 1);
    const double s_target = u * s_end;

    auto it = std::upper_bound(s.begin(), s.end(), s_target);
    const int i1 =
        std::clamp(static_cast<int>(std::distance(s.begin(), it)), 1, num_waypoints - 1);
    const int i0 = i1 - 1;

    const double s0 = s[static_cast<std::size_t>(i0)];
    const double s1 = s[static_cast<std::size_t>(i1)];
    const double a = (s_target - s0) / std::max(s1 - s0, 1e-12);

    traj_resampled.row(static_cast<int>(k)) =
        (1.0 - a) * traj_ref.row(i0) + a * traj_ref.row(i1);
  }

  return traj_resampled;
}

robot_trajectory::RobotTrajectoryPtr
toRobotTrajectory(const trajopt::TrajArray& joint_traj,
                  const moveit::core::JointModelGroup& jmg,
                  const moveit::core::RobotState& start_state, double waypoint_dt = 0.0)
{
  const std::size_t dof = jmg.getVariableCount();
  if (joint_traj.rows() < 1 || static_cast<std::size_t>(joint_traj.cols()) != dof)
  {
    throw std::invalid_argument("toRobotTrajectory: traj dims mismatch");
  }

  const auto& robot_model = start_state.getRobotModel();
  auto out =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, jmg.getName());

  moveit::core::RobotState state = start_state;
  std::vector<double> q(dof);

  for (int t = 0; t < joint_traj.rows(); ++t)
  {
    for (std::size_t j = 0; j < dof; ++j)
    {
      q[j] = joint_traj(t, static_cast<int>(j));
    }

    state.setJointGroupPositions(&jmg, q);
    state.update();
    out->addSuffixWayPoint(state, waypoint_dt);
  }

  return out;
}

}  // namespace

TrajOptInterface::TrajOptInterface(TrajOptPlannerOptions::TrajOptOptions opt)
  : options_(std::move(opt))
{
  if (options_.default_num_steps < 2)
  {
    RCLCPP_WARN(getLogger(), "trajopt.num_steps < 2 is invalid, forcing to 2");
    options_.default_num_steps = 2;
  }

  if (!options_.use_moveit_group_as_tesseract_manipulator &&
      options_.fixed_tesseract_manipulator_group.empty())
  {
    RCLCPP_WARN(getLogger(),
                "trajopt.tesseract_group is empty, forcing to 'manipulator'");
    options_.fixed_tesseract_manipulator_group = "manipulator";
  }
}

void TrajOptInterface::setRewards(const rbt_types::RewardTerms& rewards)
{
  std::scoped_lock lk(config_mutex_);
  costs_ = rbt_types::rewardToCost(rewards);
}

TrajOptInterface::StartData
TrajOptInterface::extractStart(const planning_scene::PlanningSceneConstPtr& scene,
                               const planning_interface::MotionPlanRequest& req,
                               const moveit::core::JointModelGroup& jmg) const
{
  StartData out(scene->getRobotModel());

  out.joint_names = jmg.getActiveJointModelNames();
  if (out.joint_names.empty())
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "MoveIt group '" + req.group_name + "' has no active joints");
  }

  out.variable_names = jmg.getVariableNames();
  if (out.variable_names.empty())
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "JointModelGroup has no variables");
  }

  // MoveIt idiom: merge req.start_state into the planning scene state
  const auto start_ptr = scene->getCurrentStateUpdated(req.start_state);
  if (!start_ptr)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "Failed to compute current state updated with req.start_state");
  }
  out.start_state = *start_ptr;
  out.start_state.update();

  if (!out.start_state.satisfiesBounds(&jmg))
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE,
                        "Start state violates joint bounds for group '" + jmg.getName() +
                            "'");
  }

  out.q_start.resize(static_cast<long>(out.variable_names.size()));
  out.start_state.copyJointGroupPositions(&jmg, out.q_start.data());

  return out;
}

TrajOptInterface::GoalData
TrajOptInterface::extractGoal(const planning_scene::PlanningSceneConstPtr& scene,
                              const planning_interface::MotionPlanRequest& req,
                              const moveit::core::JointModelGroup& jmg,
                              const StartData& start) const
{
  if (req.goal_constraints.empty())
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS,
                        "Request has no goal_constraints");
  }

  GoalData out(scene->getRobotModel());
  out.goal_state = start.start_state;
  out.q_goal = start.q_start;

  const std::size_t dof = jmg.getVariableCount();

  auto is_joint_only = [](const moveit_msgs::msg::Constraints& c) -> bool {
    return !c.joint_constraints.empty() && c.position_constraints.empty() &&
           c.orientation_constraints.empty() && c.visibility_constraints.empty();
  };

  // 1) Deterministic fast path: accept only "full and ordered" joint goals
  const auto& expected_names = jmg.getActiveJointModelNames();

  for (std::size_t gi = 0; gi < req.goal_constraints.size(); ++gi)
  {
    const auto& gc = req.goal_constraints[gi];
    if (!is_joint_only(gc))
    {
      continue;
    }

    if (gc.joint_constraints.size() == expected_names.size())
    {
      bool ordered = true;
      for (std::size_t j = 0; j < expected_names.size(); ++j)
      {
        if (gc.joint_constraints[j].joint_name != expected_names[j])
        {
          ordered = false;
          break;
        }
      }

      if (ordered)
      {
        // Build goal_state from the constraints
        moveit::core::RobotState goal_state(start.start_state);
        for (std::size_t j = 0; j < expected_names.size(); ++j)
        {
          goal_state.setVariablePosition(expected_names[j],
                                         gc.joint_constraints[j].position);
        }
        goal_state.update();

        if (!goal_state.satisfiesBounds(&jmg))
        {
          throw PlanningError(
              moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS,
              "Goal violates joint bounds for group '" + jmg.getName() + "'");
        }

        out.goal_index = gi;
        out.goal_raw = gc;
        out.goal_state = goal_state;

        out.q_goal.resize(static_cast<long>(dof));
        out.goal_state.copyJointGroupPositions(&jmg, out.q_goal.data());
        return out;
      }
    }
  }

  // 2) Fallback: sample a goal state from constraints (MoveIt official way)
  constraint_samplers::ConstraintSamplerManager sampler_manager;

  for (std::size_t gi = 0; gi < req.goal_constraints.size(); ++gi)
  {
    const auto& gc = req.goal_constraints[gi];
    moveit::core::RobotState goal_state(start.start_state);

    auto sampler = sampler_manager.selectSampler(scene, jmg.getName(), gc);
    if (sampler && sampler->sample(goal_state))
    {
      goal_state.update();

      if (!goal_state.satisfiesBounds(&jmg))
      {
        continue;
      }

      out.goal_index = gi;
      out.goal_raw = gc;
      out.goal_state = goal_state;

      out.q_goal.resize(static_cast<long>(dof));
      out.goal_state.copyJointGroupPositions(&jmg, out.q_goal.data());
      return out;
    }
  }

  throw PlanningError(
      moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS,
      "No goal constraint could be satisfied (neither parsed nor sampled).");
}

std::shared_ptr<const trajopt::TrajArray>
TrajOptInterface::extractTrajectorySeed(const planning_interface::MotionPlanRequest& req,
                                        const moveit::core::JointModelGroup& jmg) const
{
  const auto& waypoints = req.trajectory_constraints.constraints;
  if (waypoints.empty())
  {
    return nullptr;
  }

  if (waypoints.size() < 2)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                        "trajectory_constraints seed must contain at least 2 waypoints");
  }

  const std::vector<std::string>& joint_names = jmg.getActiveJointModelNames();
  const std::size_t dof = joint_names.size();
  if (dof == 0)
  {
    throw PlanningError(
        moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
        "trajectory_constraints seed is not supported for groups with zero DOF");
  }

  std::unordered_map<std::string, std::size_t> joint_index;
  joint_index.reserve(dof);
  for (std::size_t j = 0; j < dof; ++j)
  {
    joint_index.emplace(joint_names[j], j);
  }

  const std::size_t num_waypoints = waypoints.size();
  auto waypoint_traj = std::make_shared<trajopt::TrajArray>(
      static_cast<int>(num_waypoints), static_cast<int>(dof));

  for (std::size_t i = 0; i < num_waypoints; ++i)
  {
    const auto& waypoint = waypoints[i];

    if (!waypoint.position_constraints.empty() ||
        !waypoint.orientation_constraints.empty() ||
        !waypoint.visibility_constraints.empty())
    {
      throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                          "trajectory_constraints waypoint " + std::to_string(i) +
                              " contains non-joint constraints; only joint_constraints "
                              "are supported as a seed");
    }

    std::vector<bool> seen(dof, false);

    for (const auto& jc : waypoint.joint_constraints)
    {
      if (jc.joint_name.empty())
      {
        throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                            "trajectory_constraints waypoint " + std::to_string(i) +
                                " has JointConstraint with empty joint_name");
      }

      const auto it = joint_index.find(jc.joint_name);
      if (it == joint_index.end())
      {
        throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                            "trajectory_constraints waypoint " + std::to_string(i) +
                                " references unknown joint '" + jc.joint_name + "'");
      }

      const std::size_t col = it->second;
      if (seen[col])
      {
        throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                            "trajectory_constraints waypoint " + std::to_string(i) +
                                " has duplicate joint constraint for '" + jc.joint_name +
                                "'");
      }
      seen[col] = true;

      const double pos = jc.position;
      if (!std::isfinite(pos))
      {
        throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                            "trajectory_constraints waypoint " + std::to_string(i) +
                                " has non-finite position for joint '" + jc.joint_name +
                                "'");
      }

      (*waypoint_traj)(static_cast<int>(i), static_cast<int>(col)) = pos;
    }

    for (std::size_t col = 0; col < dof; ++col)
    {
      if (!seen[col])
      {
        throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                            "trajectory_constraints waypoint " + std::to_string(i) +
                                " is missing joint constraint for '" + joint_names[col] +
                                "'");
      }
    }
  }

  return waypoint_traj;
}

TrajOptInterface::PlanResult TrajOptInterface::plan(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    const std::shared_ptr<tesseract_environment::Environment>& env_snapshot) const
{
  const auto robot_model = planning_scene->getRobotModel();
  if (!robot_model)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "PlanningScene RobotModel is null");
  }

  const auto* jmg = robot_model->getJointModelGroup(req.group_name);
  if (!jmg)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME,
                        "Unknown group_name: '" + req.group_name + "'");
  }

  StartData start = extractStart(planning_scene, req, *jmg);
  GoalData goal = extractGoal(planning_scene, req, *jmg, start);
  auto seed = extractTrajectorySeed(req, *jmg);
  if (seed)
  {
    try
    {
      seed = std::make_shared<trajopt::TrajArray>(resampleByArcLength(
          *seed, static_cast<std::size_t>(options_.default_num_steps)));
    }
    catch (const std::exception& e)
    {
      throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
                          std::string{ "Failed to resample trajectory seed: " } +
                              e.what());
    }
  }

  const trajopt::TrajArray joint_traj =
      solveTrajOpt(req, env_snapshot, start, goal, seed);

  PlanResult out;
  out.trajectory = toRobotTrajectory(joint_traj, *jmg, start.start_state);
  moveit::core::robotStateToRobotStateMsg(start.start_state, out.start_state_msg);

  std::vector<std::size_t> invalid_indices;
  const bool path_valid =
      planning_scene->isPathValid(*out.trajectory, req.path_constraints,
                                  req.goal_constraints, req.group_name,
                                  false /* verbose */, &invalid_indices);
  if (!path_valid)
  {
    std::string msg = "Planned trajectory is invalid";
    if (!invalid_indices.empty())
    {
      msg += " (first invalid waypoint " + std::to_string(invalid_indices.front()) + ")";
    }
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN, msg);
  }

  return out;
}

trajopt::TrajArray TrajOptInterface::solveTrajOpt(
    const planning_interface::MotionPlanRequest& req,
    const std::shared_ptr<tesseract_environment::Environment>& env_snapshot,
    const StartData& start, const GoalData& goal,
    std::shared_ptr<const trajopt::TrajArray>& seed) const
{
  if (!env_snapshot)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "Tesseract environment snapshot is null");
  }

  rbt_types::CostTerms costs_copy;
  {
    std::scoped_lock lk(config_mutex_);
    costs_copy = costs_;
  }

  const std::string manipulator_group = resolveTesseractManipulatorGroup(req);
  std::vector<std::string> tesseract_joint_names;
  try
  {
    tesseract_joint_names = env_snapshot->getGroupJointNames(manipulator_group);
  }
  catch (const std::exception& e)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME,
                        std::string{ "Tesseract manipulator group '" } +
                            manipulator_group + "' not found: " + e.what());
  }

  if (tesseract_joint_names != start.joint_names)
  {
    const std::string moveit_joint_names_str = formatJointNames(start.joint_names);
    const std::string tesseract_joint_names_str = formatJointNames(tesseract_joint_names);
    RCLCPP_ERROR(getLogger(),
                 "MoveIt/Tesseract joint order mismatch. MoveIt group '%s' joints=%s, "
                 "Tesseract group '%s' joints=%s",
                 req.group_name.c_str(), moveit_joint_names_str.c_str(),
                 manipulator_group.c_str(), tesseract_joint_names_str.c_str());
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "MoveIt/Tesseract joint order mismatch for group '" +
                            req.group_name + "'");
  }

  try
  {
    rbt_planning::TrajOptSolver solver(env_snapshot);

    solver.setManipulatorGroup(manipulator_group);
    solver.setNumSteps(options_.default_num_steps);

    solver.setStartState(start.joint_names, start.q_start);
    solver.setGoalState(goal.q_goal);

    if (seed)
    {
      solver.setTrajSeed(seed);
    }

    solver.addCost(costs_copy);

    return solver.solve();
  }
  catch (const std::exception& e)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
                        std::string{ "TrajOpt solve failed: " } + e.what());
  }
}

std::string TrajOptInterface::resolveTesseractManipulatorGroup(
    const planning_interface::MotionPlanRequest& req) const
{
  if (options_.use_moveit_group_as_tesseract_manipulator)
  {
    return req.group_name;
  }

  if (options_.fixed_tesseract_manipulator_group.empty())
  {
    return "manipulator";
  }

  return options_.fixed_tesseract_manipulator_group;
}

std::optional<TrajOptInterface::SeedCacheEntry>
TrajOptInterface::getSeedHint(const std::string& key) const
{
  std::scoped_lock lk(cache_mutex_);
  auto it = seed_cache_.find(key);
  if (it == seed_cache_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

void TrajOptInterface::updateSeedHint(const std::string& key, SeedCacheEntry entry) const
{
  if (!options_.enable_seed_cache)
  {
    return;
  }

  std::scoped_lock lk(cache_mutex_);
  seed_cache_[key] = std::move(entry);
}

std::optional<TrajOptInterface::SolutionCacheEntry>
TrajOptInterface::getLastSolution(const std::string& key) const
{
  std::scoped_lock lk(cache_mutex_);
  auto it = solution_cache_.find(key);
  if (it == solution_cache_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

void TrajOptInterface::updateLastSolution(const std::string& key,
                                          SolutionCacheEntry entry) const
{
  if (!options_.enable_last_solution_cache)
  {
    return;
  }

  std::scoped_lock lk(cache_mutex_);
  solution_cache_[key] = std::move(entry);
}

}  // namespace rlc_planner
