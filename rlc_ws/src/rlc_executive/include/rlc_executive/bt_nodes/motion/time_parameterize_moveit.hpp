#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logger.hpp>

#include "rbt_types/joint_path.hpp"
#include "rbt_types/joint_trajectory.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"

namespace rlc_executive
{

/**
 * @brief Time-parameterizes a joint-space path locally using MoveIt TOTG.
 *
 * @details
 * The node borrows a MoveIt robot model, startup reference state, and planning
 * config registry for velocity and acceleration
 * scaling factors.
 */
class TimeParameterizeMoveIt final : public BT::SyncActionNode
{
public:
  using JointPath = rbt_types::JointPath;
  using JointTrajectory = rbt_types::JointTrajectory;

  /**
   * @brief Constructs a local MoveIt time-parameterization BT node.
   * @param[in] name BehaviorTree node instance name.
   * @param[in] config BehaviorTree node configuration.
   * @param[in] logger Logger used for diagnostics.
   * @param[in] robot_model Retained MoveIt robot model; must not be nullptr and
   * must outlive this BT node.
   * @param[in] reference_state Retained startup reference state; must not be
   * nullptr and must outlive this BT node.
   * @param[in] planning_configs Retained planning config registry; must not be
   * nullptr and must outlive this BT node.
   * @throws BT::LogicError If any retained pointer is nullptr.
   */
  TimeParameterizeMoveIt(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Logger logger,
                         moveit::core::RobotModelConstPtr robot_model,
                         const moveit::core::RobotState* reference_state,
                         const PlanningConfigRegistry* planning_configs);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing path input, group/config selectors,
   * timed trajectory output, and error output.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<const JointPath>>(PortKeys::kPath),
      BT::InputPort<std::string>(PortKeys::kGroupName),
      BT::InputPort<std::string>(PortKeys::kPlanningConfig),
      BT::OutputPort<std::shared_ptr<const JointTrajectory>>(PortKeys::kTrajectory),
      BT::OutputPort<std::string>(PortKeys::kError),
    };
  }

  /**
   * @brief Names of BT ports used by TimeParameterizeMoveIt.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string kPath = "path";
    static inline const std::string kGroupName = "group_name";
    static inline const std::string kPlanningConfig = "planning_config";
    static inline const std::string kTrajectory = "trajectory";
    static inline const std::string kError = "error";
  };

  /**
   * @brief Time-parameterizes the input path and writes a joint trajectory output.
   * @return `SUCCESS` when time parameterization succeeds, otherwise `FAILURE`.
   */
  BT::NodeStatus tick() override;

private:
  BT::NodeStatus failWithError(const std::string& msg);

  rclcpp::Logger logger_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::RobotState* reference_state_;
  const PlanningConfigRegistry* planning_configs_;
};

}  // namespace rlc_executive
