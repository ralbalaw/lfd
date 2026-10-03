#pragma once

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rlc_interfaces/action/collect_demonstration.hpp>

#include "rbt_types/joint_trajectory.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

/**
 * @brief Collects a joint-space demonstration through the frontend action server.
 *
 * @details
 * This BT node keeps the existing `RecordServoDemo` registration name for tree
 * compatibility, but delegates the long-running collection lifecycle to the
 * `CollectDemonstration` ROS action. The action result is converted from
 * `moveit_msgs::msg::RobotTrajectory` into `rbt_types::JointTrajectory` before
 * writing it to the blackboard.
 *
 * Input ports:
 * - `action_name`: `CollectDemonstration` action server name inherited from
 *   `BT::RosActionNode`; if omitted, the `BT::RosNodeParams` default is used.
 * - `sample_dt`: Fixed sample period [s] requested for the saved trajectory.
 *
 * Output ports:
 * - `demonstration`: Recorded joint-space trajectory [rad], shape =
 *   (num_samples, joint_dim), RowMajor.
 * - `error`: Human-readable failure text.
 * - `feedback`: Running progress text.
 * - `elapsed_sec`: Elapsed collection time [s].
 */
class RecordServoDemo final
  : public BT::RosActionNode<rlc_interfaces::action::CollectDemonstration>
{
public:
  using Action = rlc_interfaces::action::CollectDemonstration;
  using Base = BT::RosActionNode<Action>;
  using Feedback = typename Action::Feedback;
  using Goal = typename Base::Goal;
  using JointTrajectory = rbt_types::JointTrajectory;
  using WrappedResult = typename Base::WrappedResult;

  /**
   * @brief Constructs the demonstration collection BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   * @param[in] params ROS node parameters used by `BT::RosActionNode`; the ROS node
   * referenced by `params.nh` must outlive this BT node.
   */
  RecordServoDemo(const std::string& name, const BT::NodeConfig& config,
                  const BT::RosNodeParams& params);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing the action name, collection inputs,
   * demonstration output, and standard diagnostic outputs.
   */
  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
        BT::InputPort<double>(PortKeys::kSampleDt, 0.05, ""),
        BT::OutputPort<std::shared_ptr<const JointTrajectory>>(PortKeys::kDemonstration),
        BT::OutputPort<std::string>(PortKeys::kError),
        BT::OutputPort<std::string>(PortKeys::kFeedback),
        BT::OutputPort<double>(PortKeys::kElapsedSec),
    });
  }

  /**
   * @brief Names of BT ports used by RecordServoDemo.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string kSampleDt = "sample_dt";
    static inline const std::string kDemonstration = "demonstration";
  };

private:
  bool setGoal(Goal& goal) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onResultReceived(const WrappedResult& result) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;

  [[nodiscard]] double elapsedSec();

  BT::NodeStatus failWithError(const std::string& msg, double elapsed_sec = 0.0);

  std::string last_failure_message_;
  rclcpp::Time start_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Logger logger_;
};

}  // namespace rlc_executive
