#include <rlc_planner/trajopt/trajopt_planning_context.hpp>

#include <rlc_planner/exceptions.hpp>
#include <rlc_planner/trajopt/trajopt_interface.hpp>

#include <moveit/utils/moveit_error_code.hpp>

#include <chrono>
#include <exception>

namespace rlc_planner
{
namespace
{
constexpr char TRAJOPT_PLANNER_ID[] = "rlc_trajopt";

}  // namespace

TrajOptPlanningContext::TrajOptPlanningContext(const std::string& name,
                                               const std::string& group,
                                               TrajOptInterfacePtr trajopt_interface,
                                               EnvironmentPtr env_snapshot)
  : planning_interface::PlanningContext(name, group)
  , trajopt_interface_(std::move(trajopt_interface))
  , env_snapshot_(std::move(env_snapshot))
{
}

TrajOptPlanningContext::~TrajOptPlanningContext() = default;

void TrajOptPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  res.trajectory.reset();
  res.planner_id = request_.planner_id.empty() ? getName() : request_.planner_id;

  const auto t0 = std::chrono::steady_clock::now();

  try
  {
    auto planning_result =
        trajopt_interface_->plan(planning_scene_, request_, env_snapshot_);
    res.trajectory = std::move(planning_result.trajectory);
    res.start_state = std::move(planning_result.start_state_msg);
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "", TRAJOPT_PLANNER_ID);
  }
  catch (const PlanningError& e)
  {
    res.error_code = moveit::core::MoveItErrorCode(e.code(), e.what(), TRAJOPT_PLANNER_ID);
  }
  catch (const std::exception& e)
  {
    res.error_code =
        moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                                      std::string{ "Unhandled exception: " } + e.what(),
                                      TRAJOPT_PLANNER_ID);
  }

  const auto t1 = std::chrono::steady_clock::now();

  res.planning_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
}

void TrajOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  res.trajectory.clear();
  res.description.clear();
  res.processing_time.clear();

  res.planner_id = request_.planner_id.empty() ? getName() : request_.planner_id;

  const auto t0 = std::chrono::steady_clock::now();

  try
  {
    auto planning_result =
        trajopt_interface_->plan(planning_scene_, request_, env_snapshot_);
    const auto t1 = std::chrono::steady_clock::now();

    const double duration_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();

    res.trajectory.push_back(std::move(planning_result.trajectory));
    res.description.emplace_back("TrajOpt");
    res.processing_time.push_back(duration_s);

    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    res.error_code.message.clear();
    res.error_code.source = TRAJOPT_PLANNER_ID;
  }
  catch (const PlanningError& e)
  {
    res.error_code.val = e.code();
    res.error_code.message = e.what();
    res.error_code.source = TRAJOPT_PLANNER_ID;
  }
  catch (const std::exception& e)
  {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    res.error_code.message = std::string{ "Unhandled exception: " } + e.what();
    res.error_code.source = TRAJOPT_PLANNER_ID;
  }
}

bool TrajOptPlanningContext::terminate()
{
  return false;
}

void TrajOptPlanningContext::clear()
{
}

}  // namespace rlc_planner
