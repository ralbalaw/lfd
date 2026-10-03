#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/logger.hpp>
#include <tesseract_environment/environment_monitor_interface.h>

#include "rbt_planning/trajopt_planner.hpp"
#include "rbt_types/joint_endpoints.hpp"
#include "rbt_types/joint_path.hpp"
#include "rbt_types/objective_term.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/config/planning_config.hpp"

namespace rlc_executive
{

/**
 * @brief BehaviorTree node that plans a joint path with TrajOpt.
 *
 * @details
 * This BT node snapshots the monitored Tesseract environment, constructs a
 * `rbt_planning::TrajOptPlanner`, and runs planning on a worker thread. The node stays
 * in `RUNNING` until the worker finishes, then writes the planned path or a
 * failure diagnostic.
 */
class TesseractPlan final : public BT::StatefulActionNode
{
public:
  using RewardTerms = rbt_types::RewardTerms;
  using JointPath = rbt_types::JointPath;

  using JointEndpoints = rbt_types::JointEndpoints;

  using TrajOptPlanner = rbt_planning::TrajOptPlanner;

  using EnvironmentMonitorInterface = tesseract_environment::EnvironmentMonitorInterface;

  /**
   * @brief Constructs the Tesseract planning BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   * @param[in] logger Logger used for diagnostics.
   * @param[in,out] env_monitor_interface Tesseract environment monitor interface;
   * must not be nullptr and must outlive this BT node.
   * @param[in] planning_config Default planning config; must not be nullptr and
   * must outlive this BT node.
   * @param[in] monitor_namespace ROS namespace used to snapshot the Tesseract
   * environment; must not be empty.
   * @throws BT::LogicError If any retained pointer is nullptr or
   * `monitor_namespace` is empty.
   */
  TesseractPlan(const std::string& name, const BT::NodeConfig& config,
                rclcpp::Logger logger, EnvironmentMonitorInterface* env_monitor_interface,
                const PlanningConfig* planning_config,
                const std::string& monitor_namespace);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing the planning group, joint endpoints,
   * optional reward terms, planned path, and standard diagnostic outputs.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::kGroupName),
      BT::InputPort<RewardTerms>(PortKeys::kRewardTerms, RewardTerms{}, ""),
      BT::InputPort<JointEndpoints>(PortKeys::kJointEndpoints),

      BT::OutputPort<std::shared_ptr<const JointPath>>(PortKeys::kPath),
      BT::OutputPort<std::string>(PortKeys::kError),
      BT::OutputPort<std::string>(PortKeys::kFeedback),
      BT::OutputPort<double>(PortKeys::kElapsedSec),
    };
  }

  /**
   * @brief Blackboard key names used by TesseractPlan.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string kJointEndpoints = "joint_endpoints";
    static inline const std::string kGroupName = "group_name";
    static inline const std::string kPath = "path";
    static inline const std::string kRewardTerms = "reward_terms";
  };

private:
  std::shared_ptr<TrajOptPlanner> initializePlanner(const std::string& group_name);

  void runPlannerWorker(const JointEndpoints& joint_endpoints,
                        const RewardTerms& reward_terms);

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  BT::NodeStatus failWithError(const std::string& msg, double elapsed_sec = 0.0);

  std::mutex result_mutex_;
  std::jthread worker_thread_{};

  std::shared_ptr<const JointPath> path_;

  std::string error_;

  double elapsed_sec_{ 0.0 };

  bool worker_done_{ false };

  std::shared_ptr<TrajOptPlanner> planner_;

  rclcpp::Logger logger_;
  EnvironmentMonitorInterface* env_monitor_interface_;
  std::string monitor_namespace_;
  const PlanningConfig* planning_config_;
};

}  // namespace rlc_executive
