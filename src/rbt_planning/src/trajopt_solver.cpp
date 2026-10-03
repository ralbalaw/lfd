#include "rbt_planning/trajopt_solver.hpp"

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/num_diff.hpp>

namespace rbt_planning
{

TrajOptSolver::TrajOptSolver(std::shared_ptr<tesseract_environment::Environment> env)
  : env_(std::move(env))
{
}

void TrajOptSolver::setEnvironment(std::shared_ptr<tesseract_environment::Environment> env)
{
  env_ = std::move(env);
}

void TrajOptSolver::setManipulatorGroup(const std::string& group)
{
  manipulator_group_ = group;
}

void TrajOptSolver::setNumSteps(int n_steps)
{
  if (n_steps < 2)
  {
    throw std::invalid_argument("TrajOptSolver: n_steps must be >= 2");
  }
  n_steps_ = n_steps;
}

void TrajOptSolver::setStartState(const std::vector<std::string>& joint_names,
                                  const TrajOptSolver::JointVec& q_start)
{
  if (!env_)
  {
    throw std::runtime_error("TrajOptSolver: Environment is not set");
  }

  if (static_cast<int>(joint_names.size()) != q_start.size())
  {
    throw std::invalid_argument(
        "TrajOptSolver: joint_names.size() must match q_start.size()");
  }

  joint_names_ = joint_names;
  q_start_ = q_start;

  env_->setState(joint_names_, q_start_);
}

void TrajOptSolver::setGoalState(const TrajOptSolver::JointVec& q_goal)
{
  if (q_start_.size() == 0)
  {
    throw std::runtime_error("TrajOptSolver: call setStartState() first");
  }

  if (q_goal.size() != q_start_.size())
  {
    throw std::invalid_argument("TrajOptSolver: q_goal must match q_start dimension");
  }

  q_goal_ = q_goal;
}

void TrajOptSolver::validateTrajectory(
    const std::shared_ptr<const trajopt::TrajArray>& path) const
{
  if (path->rows() != n_steps_)
  {
    throw std::invalid_argument("TrajOptSolver: seed trajectory rows must match n_steps");
  }

  if (path->cols() != q_start_.size())
  {
    throw std::invalid_argument("TrajOptSolver: seed trajectory cols must match ndof");
  }
}

void TrajOptSolver::setTrajSeed(std::shared_ptr<const trajopt::TrajArray> path)
{
  validateTrajectory(path);
  path_seed_ = std::move(path);
}

void TrajOptSolver::addCost(CostTerm cost)
{
  if (!cost.feature)
  {
    throw std::invalid_argument("TrajOptSolver: cost term is invalid");
  }
  cost_terms_.push_back(std::move(cost));
}

void TrajOptSolver::addCost(const CostTerms& costs)
{
  for (const auto& cost : costs)
  {
    addCost(cost);
  }
}

trajopt::TrajArray TrajOptSolver::solve()
{
  if (!env_)
  {
    throw std::runtime_error("TrajOptSolver: Environment is not set");
  }
  if (q_start_.size() == 0 || q_goal_.size() == 0)
  {
    throw std::runtime_error("TrajOptSolver: start/goal not set");
  }

  constructProblem();

  sco::BasicTrustRegionSQP opt(prob_);
  const sco::DblVec x0 = trajopt::trajToDblVec(prob_->GetInitTraj());

  opt.initialize(x0);
  last_status_ = opt.optimize();

  return trajopt::getTraj(opt.x(), prob_->GetVars());
}

void TrajOptSolver::constructProblem()
{
  const int ndof = static_cast<int>(q_start_.size());

  trajopt::ProblemConstructionInfo pci(env_);

  pci.basic_info.n_steps = n_steps_;
  pci.basic_info.manip = manipulator_group_;
  pci.basic_info.fixed_timesteps = trajopt::IntVec{ 0 };
  pci.basic_info.use_time = false;

  pci.kin = env_->getJointGroup(manipulator_group_);

  pci.init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  if (path_seed_)
  {
    pci.init_info.data = *path_seed_;
  }
  else
  {
    pci.init_info.data = makeLinearSeed(q_start_, q_goal_, n_steps_);
  }

  auto vel = std::make_shared<trajopt::JointVelTermInfo>();
  vel->term_type = trajopt::TermType::TT_COST;
  vel->name = "joint_vel_smooth";
  vel->coeffs = trajopt::DblVec(ndof, 1.0);
  vel->targets = trajopt::DblVec(ndof, 0.0);
  pci.cost_infos.push_back(vel);

  auto goal = std::make_shared<trajopt::JointPosTermInfo>();
  goal->term_type = trajopt::TermType::TT_CNT;
  goal->name = "goal_joint_target";
  goal->first_step = n_steps_ - 1;
  goal->last_step = n_steps_ - 1;
  goal->coeffs = trajopt::DblVec(ndof, 1.0);
  goal->targets = trajopt::DblVec(q_goal_.data(), q_goal_.data() + q_goal_.size());
  pci.cnt_infos.push_back(goal);

  prob_ = trajopt::ConstructProblem(pci);
  attachCosts(*prob_);
}

sco::VarVector TrajOptSolver::getVars(trajopt::TrajOptProb& prob)
{
  sco::VarVector vars;
  const int num_steps = prob.GetNumSteps();
  const int ndof = prob.GetNumDOF();
  vars.reserve(static_cast<std::size_t>(num_steps * ndof));
  for (int t = 0; t < num_steps; ++t)
  {
    auto row = prob.GetVarRow(t);
    vars.insert(vars.end(), row.begin(), row.end());
  }
  return vars;
}

void TrajOptSolver::attachCosts(trajopt::TrajOptProb& prob) const
{
  const int num_steps = prob.GetNumSteps();
  const int dof = prob.GetNumDOF();
  auto vars = getVars(prob);

  for (const auto& cost : cost_terms_)
  {
    const std::string cost_name = cost.feature->name();
    auto f = sco::ScalarOfVector::construct(
        [num_steps, dof, c = cost](const Eigen::VectorXd& x) -> double {
          Eigen::Map<const trajopt::TrajArray> path(x.data(), num_steps, dof);
          return c.weight * c.feature->evaluate(path);
        });

    auto cost_term = std::make_shared<sco::CostFromFunc>(f, vars, cost_name);
    prob.addCost(cost_term);
  }
}

trajopt::TrajArray TrajOptSolver::makeLinearSeed(const TrajOptSolver::JointVec& q0,
                                                 const TrajOptSolver::JointVec& q1,
                                                 int n_steps) const
{
  if (n_steps < 2)
  {
    throw std::invalid_argument("TrajOptSolver: n_steps must be >= 2");
  }
  if (q0.size() != q1.size())
  {
    throw std::invalid_argument("TrajOptSolver: q0 and q1 must match");
  }

  const int ndof = static_cast<int>(q0.size());
  trajopt::TrajArray path(n_steps, ndof);

  for (int t = 0; t < n_steps; ++t)
  {
    const double a = static_cast<double>(t) / static_cast<double>(n_steps - 1);
    path.row(t) = ((1.0 - a) * q0 + a * q1).transpose();
  }
  return path;
}

}  // namespace rbt_planning
