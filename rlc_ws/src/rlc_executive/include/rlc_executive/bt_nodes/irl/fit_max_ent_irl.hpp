#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/logger.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rbt_irl/max_ent_irl.hpp"
#include "rbt_planning/trajopt_planner.hpp"
#include "rbt_types/feature.hpp"
#include "rbt_types/joint_path.hpp"
#include "rbt_types/joint_trajectory.hpp"
#include "rbt_types/math.hpp"
#include "rbt_types/objective_term.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/config/planning_config.hpp"

namespace rlc_executive
{

/**
 * @brief BehaviorTree node that fits reward terms from a demonstrated trajectory.
 */
class FitMaxEntIRL final : public BT::StatefulActionNode
{
public:
  using RewardTerms = rbt_types::RewardTerms;
  using JointPaths = rbt_types::JointPaths;
  using JointTrajectory = rbt_types::JointTrajectory;
  using JointTrajectories = rbt_types::JointTrajectories;
  using Features = rbt_types::Features;

  using WeightVec = rbt_types::WeightVec;

  using TrajOptPlanner = rbt_planning::TrajOptPlanner;

  using EnvMonitorInterface = tesseract_monitoring::ROSEnvironmentMonitorInterface;

  /**
   * @brief Constructs the MaxEnt IRL BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   * @param[in] logger Logger used for diagnostics.
   * @param[in,out] env_monitor_interface Tesseract ROS environment monitor
   * interface; must not be nullptr and must outlive this BT node.
   * @param[in] planning_config Default planning config; may be nullptr. If
   * present, it must outlive this BT node.
   * @param[in] monitor_namespace ROS namespace used to snapshot the Tesseract
   * environment; must not be empty.
   * @throws BT::RuntimeError If `env_monitor_interface` is nullptr or
   * `monitor_namespace` is empty.
   */
  FitMaxEntIRL(const std::string& name, const BT::NodeConfig& config,
               rclcpp::Logger logger, EnvMonitorInterface* env_monitor_interface,
               const PlanningConfig* planning_config,
               const std::string& monitor_namespace);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing the demonstrated trajectories, feature
   * set, optional manipulator group, fitted reward terms, and standard diagnostic
   * outputs.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::kGroupName, "", ""),
      BT::InputPort<std::shared_ptr<const JointTrajectories>>(PortKeys::kDemonstrations),
      BT::InputPort<Features>(PortKeys::kFeatures),
      BT::OutputPort<RewardTerms>(PortKeys::kRewardTerms),
      BT::OutputPort<std::string>(PortKeys::kError),
      BT::OutputPort<std::string>(PortKeys::kFeedback),
      BT::OutputPort<double>(PortKeys::kElapsedSec),
    };
  }

  /**
   * @brief Blackboard key names used by FitMaxEntIRL.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string kGroupName = "group_name";
    static inline const std::string kDemonstrations = "demonstrations";
    static inline const std::string kFeatures = "features";
    static inline const std::string kRewardTerms = "reward_terms";
  };

private:
  std::shared_ptr<TrajOptPlanner> initializePlanner(const std::string& group_name);

  void iterationCallback(const WeightVec& theta, const WeightVec& theta_prev,
                         std::chrono::milliseconds elapsed_time, int iteration);

  void runFitWorker(const std::stop_token& stop_token,
                    const std::shared_ptr<const JointTrajectories>& demo_trajectories,
                    std::shared_ptr<TrajOptPlanner> planner);

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  BT::NodeStatus failWithError(const std::string& msg, double elapsed_sec = 0.0);

  struct ProgressState
  {
    std::string feedback;
    std::string error;
    double elapsed_sec{ 0.0 };
    int iteration{ 0 };
    bool completed{ false };
    bool succeeded{ false };
    WeightVec theta;
  };

  rclcpp::Logger logger_;

  Features features_;
  std::mutex progress_mutex_;
  ProgressState progress_{};
  std::jthread worker_thread_{};

  EnvMonitorInterface* env_monitor_interface_;
  const PlanningConfig* planning_config_;
  std::string monitor_namespace_;
};

}  // namespace rlc_executive
