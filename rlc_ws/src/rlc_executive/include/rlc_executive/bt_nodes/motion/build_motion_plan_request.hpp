#pragma once

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <rclcpp/logger.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

/**
 * @brief Builds a MoveIt motion plan request from BT ports.
 *
 * @details
 * Goal input modes:
 * - Pose goal: provide `target_pose` (in `target_pose.header.frame_id`) and
 * `link_name`.
 * - Joint goal: provide both `joint_names` and `joint_positions`.
 *
 * `group_name` is always required. `link_name` is required only when using a pose
 * goal.
 */
class BuildMotionPlanRequest final : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructs the motion-plan request builder BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   * @param[in] logger Logger used for diagnostics.
   */
  BuildMotionPlanRequest(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Logger logger);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing goal inputs, tolerance inputs, request
   * output, and error output.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::kGroupName),

      BT::InputPort<geometry_msgs::msg::PoseStamped>(PortKeys::kTargetPose),
      BT::InputPort<std::string>(PortKeys::kLinkName),

      // Joint goal (useful for direct joint targets)
      BT::InputPort<std::vector<std::string>>(PortKeys::kJointNames),
      BT::InputPort<std::vector<double>>(PortKeys::kJointPositions),

      // Tolerances (used for whichever goal form is used)
      BT::InputPort<double>(PortKeys::kPositionTolerance, 0.005, ""),
      BT::InputPort<double>(PortKeys::kOrientationTolerance, 0.05, ""),
      BT::InputPort<double>(PortKeys::kJointTolerance, 0.001, ""),

      // Output
      BT::OutputPort<std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest>>(
          PortKeys::kRequest),

      // Optional: debugging string
      BT::OutputPort<std::string>(PortKeys::kError),
    };
  }

  /**
   * @brief Names of BT ports used by BuildMotionPlanRequest.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string kGroupName = "group_name";
    static inline const std::string kTargetPose = "target_pose";
    static inline const std::string kLinkName = "link_name";
    static inline const std::string kJointNames = "joint_names";
    static inline const std::string kJointPositions = "joint_positions";
    static inline const std::string kPositionTolerance = "position_tolerance";
    static inline const std::string kOrientationTolerance = "orientation_tolerance";
    static inline const std::string kJointTolerance = "joint_tolerance";
    static inline const std::string kRequest = "request";
  };

  /**
   * @brief Builds a MoveIt motion-plan request and writes it to the output port.
   * @return `SUCCESS` if a valid pose or joint goal was converted; otherwise `FAILURE`.
   */
  BT::NodeStatus tick() override;

private:
  BT::NodeStatus failWithError(const std::string& msg);

  rclcpp::Logger logger_;
};

}  // namespace rlc_executive
