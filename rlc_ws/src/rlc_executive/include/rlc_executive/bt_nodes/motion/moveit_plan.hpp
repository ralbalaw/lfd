#pragma once

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <rclcpp/logger.hpp>

#include "rbt_types/joint_trajectory.hpp"
#include "rlc_executive/bt_nodes/common/port_types.hpp"
#include "rlc_executive/config/planning_config_registry.hpp"

namespace rlc_executive
{

/**
 * @brief Starts MoveIt planning through BehaviorTree.ROS2's action wrapper.
 *
 * @details
 * This BT node reads a motion plan request and planning config from input ports,
 * fills a MoveGroup action goal, and delegates the ROS action-client lifecycle to
 * `BT::RosActionNode`.
 *
 * Input ports:
 * - `action_name`: MoveGroup action server name, inherited from
 *   `BT::RosActionNode`; if omitted, the `BT::RosNodeParams` default is used.
 * - `request`: Motion plan request to send to MoveIt.
 * - `planning_config`: Planning config name (selects planner pipeline and knobs).
 *
 * Output ports:
 * - `plan_result`: Planning result summary (status + message + error_code + timing).
 * - `trajectory`: Planned joint-space trajectory on success.
 */
class MoveItPlan final : public BT::RosActionNode<moveit_msgs::action::MoveGroup>
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using Base = BT::RosActionNode<MoveGroup>;
  using Goal = typename Base::Goal;
  using WrappedResult = typename Base::WrappedResult;

  /**
   * @brief Creates the ROS2-backed MoveIt planning BT node.
   * @param[in] name BT node instance name from the XML tree.
   * @param[in] config BT node configuration, including ports and blackboard.
   * @param[in] params ROS node parameters used by `BT::RosActionNode`; the ROS
   * node referenced by `params.nh` must outlive this BT node.
   * @param[in] planning_configs Retained planning config registry; must not be
   * nullptr and must outlive this BT node.
   */
  MoveItPlan(const std::string& name, const BT::NodeConfig& config,
             const BT::RosNodeParams& params,
             const PlanningConfigRegistry* planning_configs);

  /// @brief Declares the ports used by this BT node.
  /// @return Port list for XML registration.
  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
        BT::InputPort<std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest>>(
            PortKeys::kRequest),
        BT::InputPort<std::string>(PortKeys::kPlanningConfig),
        BT::OutputPort<PlanSummary>(PortKeys::kPlanResult),
        BT::OutputPort<std::shared_ptr<const rbt_types::JointTrajectory>>(
            PortKeys::kTrajectory),
    });
  }

  /**
   * @brief Names of BT ports used by MoveItPlan.
   */
  struct PortKeys
  {
    static inline const std::string kRequest = "request";
    static inline const std::string kPlanningConfig = "planning_config";
    static inline const std::string kPlanResult = "plan_result";
    static inline const std::string kTrajectory = "trajectory";
  };

private:
  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& result) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;

  BT::NodeStatus failWithError(const std::string& msg,
                               PlanStatus status = PlanStatus::kFailure,
                               int moveit_error_code = 0, double planning_time_sec = 0.0);

  std::string config_name_;
  std::string last_failure_message_;

  const PlanningConfigRegistry* planning_configs_;

  rclcpp::Logger logger_;
};

}  // namespace rlc_executive
