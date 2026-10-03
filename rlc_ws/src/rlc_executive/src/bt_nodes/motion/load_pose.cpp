#include "rlc_executive/bt_nodes/motion/load_pose.hpp"

#include <exception>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

LoadPose::LoadPose(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Logger logger)
  : BT::SyncActionNode(name, config)
  , logger_(logger.get_child(bt_utils::sanitizeLoggerName(fullPath())))
{
}

BT::NodeStatus LoadPose::tick()
{
  try
  {
    const std::string frame_id =
        bt_utils::requireInput<std::string>(*this, PortKeys::kFrameId);
    const std::vector<double> position =
        bt_utils::requireInput<std::vector<double>>(*this, PortKeys::kPosition);
    const std::vector<double> orientation_xyzw =
        bt_utils::requireInput<std::vector<double>>(*this, PortKeys::kOrientationXyzw);

    if (position.size() != 3)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: '",
                             PortKeys::kPosition, "' must have length 3");
    }
    if (orientation_xyzw.size() != 4)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: '",
                             PortKeys::kOrientationXyzw, "' must have length 4");
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame_id;

    out.pose.position.x = position[0];
    out.pose.position.y = position[1];
    out.pose.position.z = position[2];

    out.pose.orientation.x = orientation_xyzw[0];
    out.pose.orientation.y = orientation_xyzw[1];
    out.pose.orientation.z = orientation_xyzw[2];
    out.pose.orientation.w = orientation_xyzw[3];

    bt_utils::setOutput(*this, PortKeys::kTargetPose, out);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus LoadPose::failWithError(const std::string& msg)
{
  bt_utils::setOutput(*this, PortKeys::kTargetPose, geometry_msgs::msg::PoseStamped());
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}
}  // namespace rlc_executive
