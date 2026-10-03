#include "rbt_planning/ompl_solver.hpp"

#include <stdexcept>
#include <unordered_map>
#include <mutex>
#include <thread>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// Tesseract
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_common/eigen_types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

namespace rbt_planning
{
namespace
{

class TesseractStateValidityChecker final : public ompl::base::StateValidityChecker
{
public:
  TesseractStateValidityChecker(
      const ompl::base::SpaceInformationPtr& si,
      std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
      std::shared_ptr<tesseract_collision::DiscreteContactManager> base_cm,
      std::vector<std::string> active_links, std::size_t dof)
    : ompl::base::StateValidityChecker(si)
    , manip_(std::move(manip))
    , base_cm_(std::move(base_cm))
    , links_(std::move(active_links))
    , dof_(dof)
  {
  }

  bool isValid(const ompl::base::State* state) const override
  {
    const auto tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
    std::shared_ptr<tesseract_collision::DiscreteContactManager> cm;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = cm_cache_.find(tid);
      if (it == cm_cache_.end())
      {
        // clone() is typically UPtr; wrap into shared_ptr for caching.
        auto u = base_cm_->clone();
        cm = std::shared_ptr<tesseract_collision::DiscreteContactManager>(std::move(u));
        cm_cache_.emplace(tid, cm);
      }
      else
      {
        cm = it->second;
      }
    }

    // OMPL RealVector -> Eigen
    const auto* rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::Map<const Eigen::VectorXd> q(rv->values, static_cast<Eigen::Index>(dof_));

    // FK for active links
    const tesseract_common::TransformMap link_tf = manip_->calcFwdKin(
        q);  // returns map<name, pose> :contentReference[oaicite:1]{index=1}
    for (const auto& link_name : links_)
    {
      auto it = link_tf.find(link_name);
      if (it != link_tf.end())
      {
        cm->setCollisionObjectsTransform(link_name, it->second);
      }
    }

    // Collision test
    tesseract_collision::ContactResultMap contact_map;
    cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);
    return contact_map.empty();
  }

private:
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  std::shared_ptr<tesseract_collision::DiscreteContactManager> base_cm_;
  std::vector<std::string> links_;
  std::size_t dof_;

  mutable std::mutex mutex_;
  mutable std::unordered_map<std::size_t,
                             std::shared_ptr<tesseract_collision::DiscreteContactManager>>
      cm_cache_;
};

static void ensureEnv(const std::shared_ptr<tesseract_environment::Environment>& env)
{
  if (!env)
  {
    throw std::runtime_error("OMPLSolver: Environment is not set");
  }
}

}  // namespace

OMPLSolver::OMPLSolver(std::shared_ptr<tesseract_environment::Environment> env)
  : env_(std::move(env))
{
}

void OMPLSolver::setEnvironment(std::shared_ptr<tesseract_environment::Environment> env)
{ env_ = std::move(env); }

void OMPLSolver::setManipulatorGroup(const std::string& group)
{ manipulator_group_ = group; }

void OMPLSolver::setStartState(const std::vector<std::string>& joint_names,
                               const OMPLSolver::JointVec& q_start)
{
  ensureEnv(env_);
  if (static_cast<Eigen::Index>(joint_names.size()) != q_start.size())
  {
    throw std::invalid_argument("OMPLSolver: joint_names.size() != q_start.size()");
  }

  joint_names_ = joint_names;
  q_start_ = q_start;

  env_->setState(joint_names_, q_start_);
}

void OMPLSolver::setGoalState(const OMPLSolver::JointVec& q_goal)
{
  if (q_start_.size() == 0)
  {
    throw std::runtime_error("OMPLSolver: setStartState() must be called first");
  }
  if (q_goal.size() != q_start_.size())
  {
    throw std::invalid_argument("OMPLSolver: q_goal.size() != q_start.size()");
  }
  q_goal_ = q_goal;
}

void OMPLSolver::setPlanningTime(double seconds)
{
  if (seconds <= 0.0)
  {
    throw std::invalid_argument("OMPLSolver: planning time must be > 0");
  }
  planning_time_ = seconds;
}

void OMPLSolver::setSimplify(bool enabled)
{ simplify_ = enabled; }

void OMPLSolver::setLongestValidSegmentFraction(double fraction)
{
  if (fraction <= 0.0 || fraction > 1.0)
  {
    throw std::invalid_argument(
        "OMPLSolver: longest valid segment fraction must be in (0, 1]");
  }
  lvs_fraction_ = fraction;
}

void OMPLSolver::setContactManagerConfig(
    const tesseract_collision::ContactManagerConfig& cfg)
{ contact_cfg_ = cfg; }

void OMPLSolver::setPlannerFactory(PlannerFactoryFn fn)
{ planner_factory_ = std::move(fn); }

void OMPLSolver::attachValidityChecker(
    const std::shared_ptr<ompl::geometric::SimpleSetup>& ss)
{
  auto manip = env_->getJointGroup(manipulator_group_);
  const std::size_t dof = static_cast<std::size_t>(q_start_.size());

  auto base_cm_u = env_->getDiscreteContactManager();
  auto base_cm =
      std::shared_ptr<tesseract_collision::DiscreteContactManager>(std::move(base_cm_u));

  const std::vector<std::string> active_links = manip->getActiveLinkNames();
  base_cm->setActiveCollisionObjects(active_links);
  base_cm->applyContactManagerConfig(contact_cfg_);

  auto checker = std::make_shared<TesseractStateValidityChecker>(
      ss->getSpaceInformation(), manip, base_cm, active_links, dof);
  ss->setStateValidityChecker(checker);
}

std::shared_ptr<ompl::geometric::SimpleSetup> OMPLSolver::buildProblem()
{
  auto manip = env_->getJointGroup(manipulator_group_);
  if (!manip)
  {
    throw std::runtime_error(
        "OMPLSolver: env->getJointGroup() returned null for group: " + manipulator_group_);
  }

  const std::size_t dof = static_cast<std::size_t>(q_start_.size());

  // Bounds from Tesseract joint limits
  const auto limits = manip->getLimits().joint_limits;
  if (static_cast<std::size_t>(limits.rows()) != dof)
  {
    throw std::runtime_error(
        "OMPLSolver: joint limits rows != dof (check group / joint ordering)");
  }

  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(dof);
  ompl::base::RealVectorBounds bounds(dof);
  for (std::size_t i = 0; i < dof; ++i)
  {
    bounds.setLow(i, limits(static_cast<Eigen::Index>(i), 0));
    bounds.setHigh(i, limits(static_cast<Eigen::Index>(i), 1));
  }
  space->setBounds(bounds);
  space->setLongestValidSegmentFraction(lvs_fraction_);

  auto ss = std::make_shared<ompl::geometric::SimpleSetup>(space);
  attachValidityChecker(ss);

  // Start and goal
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);

  auto* s = start->as<ompl::base::RealVectorStateSpace::StateType>();
  auto* g = goal->as<ompl::base::RealVectorStateSpace::StateType>();
  for (std::size_t i = 0; i < dof; ++i)
  {
    s->values[i] = q_start_(static_cast<Eigen::Index>(i));
    g->values[i] = q_goal_(static_cast<Eigen::Index>(i));
  }
  ss->setStartAndGoalStates(start, goal);

  // Planner selection (native OMPL)
  if (planner_factory_)
  {
    ss->setPlanner(planner_factory_(ss->getSpaceInformation()));
  }
  else
  {
    ss->setPlanner(
        std::make_shared<ompl::geometric::RRTConnect>(ss->getSpaceInformation()));
  }

  ss->setup();
  return ss;
}

std::shared_ptr<ompl::geometric::SimpleSetup> OMPLSolver::solve()
{
  ensureEnv(env_);
  if (q_start_.size() == 0 || q_goal_.size() == 0)
  {
    throw std::runtime_error("OMPLSolver: start/goal not set");
  }

  auto ss = buildProblem();

  const auto status = ss->solve(planning_time_);
  if (status && simplify_)
  {
    ss->simplifySolution();
  }

  return ss;
}

}  // namespace rbt_planning
