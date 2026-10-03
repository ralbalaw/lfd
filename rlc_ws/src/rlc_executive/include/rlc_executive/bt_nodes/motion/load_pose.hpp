#pragma once

#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logger.hpp>

namespace rlc_executive
{

/**
 * @brief Builds a target pose from BehaviorTree input ports.
 *
 * @details
 * Input ports:
 * - `frame_id`: Reference frame for the target pose.
 * - `position`: Position [m], size = 3, order `[x, y, z]`.
 * - `orientation_xyzw`: Quaternion, size = 4, order `[x, y, z, w]`.
 *
 * Output ports:
 * - `target_pose`: Pose built from the input ports.
 *
 * @note The output stamp is default-initialized because this node has no ROS
 * clock dependency.
 */
class LoadPose final : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructs the pose-loading BT node.
   * @param[in] name BT node instance name from the XML tree.
   * @param[in] config BehaviorTree node configuration, including ports.
   * @param[in] logger Logger used for failure diagnostics.
   */
  LoadPose(const std::string& name, const BT::NodeConfig& config, rclcpp::Logger logger);

  /// @brief Declares the ports used by this BT node.
  /// @return Port list for XML registration.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::kFrameId),
      BT::InputPort<std::vector<double>>(PortKeys::kPosition),
      BT::InputPort<std::vector<double>>(PortKeys::kOrientationXyzw),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(PortKeys::kTargetPose),
    };
  }

  /// @brief Builds and outputs the target pose.
  /// @return `SUCCESS` if inputs are valid; otherwise `FAILURE`.
  BT::NodeStatus tick() override;

  /**
   * @brief Names of BT ports used by LoadPose.
   */
  struct PortKeys
  {
    static inline const std::string kFrameId = "frame_id";
    static inline const std::string kPosition = "position";
    static inline const std::string kOrientationXyzw = "orientation_xyzw";
    static inline const std::string kTargetPose = "target_pose";
  };

private:
  BT::NodeStatus failWithError(const std::string& msg);

  rclcpp::Logger logger_;
};

}  // namespace rlc_executive
