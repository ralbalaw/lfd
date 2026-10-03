#include "rlc_executive/bt_nodes/irl/fit_max_ent_irl.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{
namespace
{
FitMaxEntIRL::RewardTerms makeRewardTerms(const FitMaxEntIRL::Features& features,
                                          const FitMaxEntIRL::WeightVec& theta)
{
  FitMaxEntIRL::RewardTerms reward_terms;
  reward_terms.reserve(features.size());

  for (std::size_t i = 0; i < features.size(); ++i)
  {
    reward_terms.push_back({ features[i], theta[static_cast<Eigen::Index>(i)] });
  }

  return reward_terms;
}

std::string makeIterationFeedback(int iteration, double step_norm)
{
  std::ostringstream stream;
  stream << "fitting max-ent irl: iteration " << (iteration + 1)
         << " (step_norm=" << step_norm << ")";
  return stream.str();
}

}  // namespace

FitMaxEntIRL::FitMaxEntIRL(const std::string& name, const BT::NodeConfig& config,
                           rclcpp::Logger logger,
                           EnvMonitorInterface* env_monitor_interface,
                           const PlanningConfig* planning_config,
                           const std::string& monitor_namespace)
  : BT::StatefulActionNode(name, config)
  , logger_(logger.get_child(bt_utils::sanitizeLoggerName(fullPath())))
  , env_monitor_interface_(env_monitor_interface)
  , planning_config_(planning_config)
  , monitor_namespace_(monitor_namespace)
{
  if (!env_monitor_interface_)
  {
    throw BT::RuntimeError(registrationName(), "[", fullPath(),
                           "]: env_monitor_interface is null");
  }
  if (monitor_namespace_.empty())
  {
    throw BT::RuntimeError(registrationName(), "[", fullPath(),
                           "]: monitor_namespace is empty");
  }
}

std::shared_ptr<FitMaxEntIRL::TrajOptPlanner>
FitMaxEntIRL::initializePlanner(const std::string& group_name)
{
  auto env_uptr = env_monitor_interface_->getEnvironment(monitor_namespace_);
  if (!env_uptr)
  {
    throw BT::RuntimeError(registrationName(), "[", fullPath(),
                           "]: failed to snapshot Tesseract environment from namespace '",
                           monitor_namespace_, "'");
  }

  std::shared_ptr<tesseract_environment::Environment> env(env_uptr.release());

  TrajOptPlanner::Options planner_opt;
  if (planning_config_)
  {
    planner_opt.ompl_planning_time = planning_config_->allowed_planning_time_sec;
  }
  planner_opt.use_ompl_seed = true;
  planner_opt.trajopt_num_steps = 20;

  return std::make_shared<TrajOptPlanner>(env, group_name, planner_opt);
}

void FitMaxEntIRL::iterationCallback(const WeightVec& theta, const WeightVec& theta_prev,
                                     std::chrono::milliseconds elapsed_time,
                                     int iteration)
{
  const double step_norm = (theta - theta_prev).norm();
  const std::string feedback = makeIterationFeedback(iteration, step_norm);
  const double elapsed_sec = std::chrono::duration<double>(elapsed_time).count();

  std::scoped_lock lock(progress_mutex_);
  progress_.feedback = feedback;
  progress_.elapsed_sec = elapsed_sec;
  progress_.iteration = iteration + 1;
}

void FitMaxEntIRL::runFitWorker(
    const std::stop_token& stop_token,
    const std::shared_ptr<const JointTrajectories>& demo_trajectories,
    std::shared_ptr<TrajOptPlanner> planner)
{
  const auto started_at = std::chrono::steady_clock::now();

  try
  {
    JointPaths demo_paths;
    demo_paths.reserve(demo_trajectories->size());
    for (const JointTrajectory& trajectory : *demo_trajectories)
    {
      demo_paths.push_back(trajectory.path());
    }

    auto sampler = [planner = std::move(planner)](
                       const rbt_types::JointEndpointsSet& endpoints_set,
                       const rbt_types::RewardTerms& rewards) -> rbt_types::JointPaths {
      return planner->plan(endpoints_set, rewards);
    };
    auto iteration_callback = [this](const WeightVec& theta, const WeightVec& theta_prev,
                                     std::chrono::milliseconds elapsed_time,
                                     int iteration) {
      iterationCallback(theta, theta_prev, elapsed_time, iteration);
    };

    rbt_irl::MaxEntIRLOptions options;
    options.learning_rate = 0.99;
    options.convergence_tol = 0.01;
    options.max_iters = 100;

    WeightVec theta = rbt_irl::maxEntIRL(demo_paths, features_, sampler,
                                         iteration_callback, options, {}, stop_token);

    const bool was_canceled = stop_token.stop_requested();
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(progress_mutex_);
    progress_.elapsed_sec = elapsed_sec;
    progress_.completed = true;
    progress_.succeeded = !was_canceled;
    progress_.feedback =
        was_canceled ? "max-ent irl fit canceled" : "max-ent irl fit complete";
    progress_.error = was_canceled ? "max-ent irl fit canceled" : "";
    progress_.theta = std::move(theta);
  }
  catch (const std::exception& e)
  {
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(progress_mutex_);
    progress_.elapsed_sec = elapsed_sec;
    progress_.completed = true;
    progress_.succeeded = false;
    progress_.feedback.clear();
    progress_.error = e.what();
  }
}

BT::NodeStatus FitMaxEntIRL::onStart()
{
  try
  {
    if (worker_thread_.joinable())
    {
      worker_thread_.request_stop();
      worker_thread_.join();
    }

    bt_utils::setOutput(*this, PortKeys::kRewardTerms, RewardTerms{});

    const auto demo_trajectories =
        bt_utils::requireInput<std::shared_ptr<const JointTrajectories>>(
            *this, PortKeys::kDemonstrations);
    if (!demo_trajectories || demo_trajectories->empty())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::kDemonstrations, "' is null or empty");
    }

    features_ = bt_utils::requireInput<Features>(*this, PortKeys::kFeatures);
    const std::string group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::kGroupName);

    auto planner = initializePlanner(group_name);

    {
      std::scoped_lock lock(progress_mutex_);
      progress_ = ProgressState{};
      progress_.feedback = "starting max-ent irl fit";
    }

    bt_utils::setRunningDiagnostics(*this, "starting max-ent irl fit", 0.0);

    worker_thread_ = std::jthread([this, demo_trajectories, planner = std::move(planner)](
                                      const std::stop_token& stop_token) mutable {
      runFitWorker(stop_token, demo_trajectories, std::move(planner));
    });

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus FitMaxEntIRL::onRunning()
{
  try
  {
    ProgressState snapshot;
    {
      std::scoped_lock lock(progress_mutex_);
      snapshot = progress_;
    }

    const std::string feedback =
        snapshot.feedback.empty() ? "fitting max-ent irl" : snapshot.feedback;
    bt_utils::setRunningDiagnostics(*this, feedback, snapshot.elapsed_sec);

    if (!snapshot.completed)
    {
      return BT::NodeStatus::RUNNING;
    }

    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }

    if (!snapshot.succeeded)
    {
      return failWithError(snapshot.error.empty() ? "max-ent irl fit failed" :
                                                    snapshot.error,
                           snapshot.elapsed_sec);
    }

    bt_utils::setOutput(*this, PortKeys::kRewardTerms,
                        makeRewardTerms(features_, snapshot.theta));

    bt_utils::setSuccessDiagnostics(*this, snapshot.elapsed_sec);
    RCLCPP_INFO(logger_, "SUCCESS (%.3fs, iterations=%d)", snapshot.elapsed_sec,
                snapshot.iteration);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void FitMaxEntIRL::onHalted()
{
  if (worker_thread_.joinable())
  {
    worker_thread_.request_stop();
  }

  RCLCPP_WARN(logger_, "halt requested, canceling");
}

BT::NodeStatus FitMaxEntIRL::failWithError(const std::string& msg, double elapsed_sec)
{
  bt_utils::setOutput(*this, PortKeys::kRewardTerms, RewardTerms{});
  bt_utils::setFailureDiagnostics(*this, msg, elapsed_sec);
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace rlc_executive
