#pragma once

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include "rbt_types/joint_trajectory.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"
#include "rlc_executive/bt_nodes/common/port_types.hpp"

namespace rlc_executive
{

/**
 * @brief Executes a planned trajectory through BehaviorTree.ROS2's action wrapper.
 *
 * @details
 * This BT node reads a timed MoveIt trajectory from the input ports, fills an
 * ExecuteTrajectory action goal, and delegates the ROS action-client lifecycle to
 * `BT::RosActionNode`.
 *
 * Input ports:
 * - `action_name`: ExecuteTrajectory action server name, inherited from
 *   `BT::RosActionNode`; if omitted, the `BT::RosNodeParams` default is used.
 * - `trajectory`: Trajectory to execute.
 * - `planning_config`: Planning config name kept for logs.
 *
 * Output ports:
 * - `exec_result`: Execution result (status + message + error_code + timing).
 * - `error`: Convenience string for introspection; set to a human-readable failure
 * message on failure/cancel and cleared on success.
 * - `feedback`: Convenience string for introspection; set when the goal is sent and
 * cleared on completion.
 * - `elapsed_sec`: Convenience value for introspection; execution time [s] on
 * completion or failure.
 */
class ExecuteTrajectory final
  : public BT::RosActionNode<moveit_msgs::action::ExecuteTrajectory>
{
public:
  using Action = moveit_msgs::action::ExecuteTrajectory;
  using Base = BT::RosActionNode<Action>;
  using Goal = typename Base::Goal;
  using WrappedResult = typename Base::WrappedResult;

  /**
   * @brief Creates the ROS2-backed trajectory execution BT node.
   * @param[in] name BT node instance name from the XML tree.
   * @param[in] config BT node configuration, including ports and blackboard.
   * @param[in] params ROS node parameters used by `BT::RosActionNode`; the ROS
   * node referenced by `params.nh` must outlive this BT node.
   */
  ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config,
                    const BT::RosNodeParams& params,
                    moveit::core::RobotModelConstPtr robot_model);

  /// @brief Declares the ports used by this BT node.
  /// @return Port list for XML registration.
  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
        BT::InputPort<std::shared_ptr<const rbt_types::JointTrajectory>>(
            PortKeys::kTrajectory),
        BT::InputPort<std::string>(PortKeys::kGroupName),
        BT::InputPort<std::string>(PortKeys::kPlanningConfig),
        BT::OutputPort<ExecResult>(PortKeys::kExecResult),
        BT::OutputPort<std::string>(PortKeys::kError),
        BT::OutputPort<std::string>(PortKeys::kFeedback),
        BT::OutputPort<double>(PortKeys::kElapsedSec),
    });
  }

  /**
   * @brief Names of BT ports used by ExecuteTrajectory.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string kTrajectory = "trajectory";
    static inline const std::string kGroupName = "group_name";
    static inline const std::string kPlanningConfig = "planning_config";
    static inline const std::string kExecResult = "exec_result";
  };

private:
  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& result) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;

  void ensureNodeAndLogger();

  double elapsedSec();

  BT::NodeStatus failWithError(const std::string& msg,
                               ExecStatus status = ExecStatus::kFailure,
                               int moveit_error_code = 0,
                               double execution_time_sec = 0.0);

  std::string config_name_;
  std::string last_failure_message_;

  rclcpp::Time start_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Logger logger_;

  moveit::core::RobotModelConstPtr robot_model_;
};

}  // namespace rlc_executive
