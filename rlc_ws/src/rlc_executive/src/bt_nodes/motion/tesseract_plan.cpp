#include "rlc_executive/bt_nodes/motion/tesseract_plan.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

TesseractPlan::TesseractPlan(const std::string& name, const BT::NodeConfig& config,
                             rclcpp::Logger logger,
                             EnvironmentMonitorInterface* env_monitor_interface,
                             const PlanningConfig* planning_config,
                             const std::string& monitor_namespace)
  : BT::StatefulActionNode(name, config)
  , logger_(logger.get_child(bt_utils::sanitizeLoggerName(fullPath())))
  , env_monitor_interface_(env_monitor_interface)
  , monitor_namespace_(monitor_namespace)
  , planning_config_(planning_config)
{
  if (!env_monitor_interface_)
  {
    throw BT::LogicError(fullPath(), "]: EnvironmentMonitorInterface pointer is null");
  }
  if (!planning_config_)
  {
    throw BT::LogicError(fullPath(), "]: PlanningConfig pointer is null");
  }
  if (monitor_namespace_.empty())
  {
    throw BT::LogicError(fullPath(), "]: monitor_namespace is empty");
  }
}

std::shared_ptr<TesseractPlan::TrajOptPlanner>
TesseractPlan::initializePlanner(const std::string& group_name)
{
  auto env_uptr = env_monitor_interface_->getEnvironment(monitor_namespace_);
  if (!env_uptr)
  {
    throw BT::LogicError(registrationName(), "[", fullPath(),
                         "]: failed to snapshot Tesseract environment from namespace '",
                         monitor_namespace_, "'");
  }

  std::shared_ptr<tesseract_environment::Environment> env(env_uptr.release());

  TrajOptPlanner::Options planner_opt;
  planner_opt.ompl_planning_time = planning_config_->allowed_planning_time_sec;
  planner_opt.use_ompl_seed = true;
  planner_opt.trajopt_num_steps = 20;

  return std::make_shared<TrajOptPlanner>(env, group_name, planner_opt);
}

void TesseractPlan::runPlannerWorker(const JointEndpoints& joint_endpoints,
                                     const RewardTerms& reward_terms)
{
  const auto started_at = std::chrono::steady_clock::now();

  try
  {
    auto path = planner_->plan(joint_endpoints, reward_terms);

    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(result_mutex_);
    path_ = std::make_shared<JointPath>(std::move(path));
    error_.clear();
    elapsed_sec_ = elapsed_sec;
    worker_done_ = true;
  }
  catch (const std::exception& e)
  {
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(result_mutex_);
    path_.reset();
    error_ = e.what();
    elapsed_sec_ = elapsed_sec;
    worker_done_ = true;
  }
}

BT::NodeStatus TesseractPlan::onStart()
{
  try
  {
    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }

    const auto joint_endpoints =
        bt_utils::requireInput<JointEndpoints>(*this, PortKeys::kJointEndpoints);

    const auto reward_terms =
        bt_utils::requireInput<RewardTerms>(*this, PortKeys::kRewardTerms);

    const std::string group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::kGroupName);

    planner_ = initializePlanner(group_name);

    {
      std::scoped_lock lock(result_mutex_);
      path_.reset();
      error_.clear();
      elapsed_sec_ = 0.0;
      worker_done_ = false;
    }
    bt_utils::setOutput(*this, PortKeys::kPath, std::shared_ptr<const JointPath>());

    worker_thread_ = std::jthread(&TesseractPlan::runPlannerWorker, this, joint_endpoints,
                                  reward_terms);

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus TesseractPlan::onRunning()
{
  try
  {
    std::shared_ptr<const JointPath> path;
    std::string error;
    double elapsed_sec = 0.0;
    bool worker_done = false;
    {
      std::scoped_lock lock(result_mutex_);
      path = path_;
      error = error_;
      elapsed_sec = elapsed_sec_;
      worker_done = worker_done_;
    }

    if (!worker_done)
    {
      return BT::NodeStatus::RUNNING;
    }

    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }

    if (!error.empty())
    {
      return failWithError(error, elapsed_sec);
    }

    if (!path)
    {
      return failWithError("planner failed to produce a path", elapsed_sec);
    }

    bt_utils::setOutput(*this, PortKeys::kPath, path);

    bt_utils::setSuccessDiagnostics(*this, elapsed_sec);
    RCLCPP_INFO(logger_, "SUCCESS (%.3fs) path_points=%zu path_joints=%zu", elapsed_sec,
                path->length(), path->dim());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void TesseractPlan::onHalted()
{
  RCLCPP_WARN(logger_, "halt requested, planner will finish in background");
}

BT::NodeStatus TesseractPlan::failWithError(const std::string& msg, double elapsed_sec)
{
  bt_utils::setOutput(*this, PortKeys::kPath, std::shared_ptr<const JointPath>());
  bt_utils::setFailureDiagnostics(*this, msg, elapsed_sec);
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}
}  // namespace rlc_executive
