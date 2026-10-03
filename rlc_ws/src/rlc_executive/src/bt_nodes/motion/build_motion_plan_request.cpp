#include "rlc_executive/bt_nodes/motion/build_motion_plan_request.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{
namespace
{

moveit_msgs::msg::Constraints
makePoseGoalConstraints(const geometry_msgs::msg::PoseStamped& target_pose,
                        const std::string& link_name, double position_tol,
                        double orientation_tol)
{
  moveit_msgs::msg::Constraints c;

  moveit_msgs::msg::PositionConstraint pc;
  pc.header = target_pose.header;
  pc.link_name = link_name;
  pc.weight = 1.0;

  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions.resize(1);
  sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = position_tol;

  geometry_msgs::msg::Pose sphere_pose;
  sphere_pose.position = target_pose.pose.position;
  sphere_pose.orientation.w = 1.0;

  pc.constraint_region.primitives.push_back(sphere);
  pc.constraint_region.primitive_poses.push_back(sphere_pose);

  moveit_msgs::msg::OrientationConstraint oc;
  oc.header = target_pose.header;
  oc.link_name = link_name;
  oc.orientation = target_pose.pose.orientation;
  oc.absolute_x_axis_tolerance = orientation_tol;
  oc.absolute_y_axis_tolerance = orientation_tol;
  oc.absolute_z_axis_tolerance = orientation_tol;
  oc.weight = 1.0;

  c.position_constraints.push_back(std::move(pc));
  c.orientation_constraints.push_back(std::move(oc));
  return c;
}

moveit_msgs::msg::Constraints
makeJointGoalConstraints(const std::vector<std::string>& joint_names,
                         const std::vector<double>& joint_positions, double joint_tol)
{
  moveit_msgs::msg::Constraints c;
  c.joint_constraints.reserve(joint_names.size());

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = joint_names[i];
    jc.position = joint_positions[i];
    jc.tolerance_above = joint_tol;
    jc.tolerance_below = joint_tol;
    jc.weight = 1.0;
    c.joint_constraints.push_back(std::move(jc));
  }

  return c;
}

}  // namespace

BuildMotionPlanRequest::BuildMotionPlanRequest(const std::string& name,
                                               const BT::NodeConfig& config,
                                               rclcpp::Logger logger)
  : BT::SyncActionNode(name, config)
  , logger_(logger.get_child(bt_utils::sanitizeLoggerName(fullPath())))
{
}

BT::NodeStatus BuildMotionPlanRequest::tick()
{
  try
  {
    auto req = std::make_shared<moveit_msgs::msg::MotionPlanRequest>();

    req->group_name = bt_utils::requireInput<std::string>(*this, PortKeys::kGroupName);
    // Use the current state as the start state unless the caller provides a diff.
    req->start_state.is_diff = true;

    const double pos_tol =
        bt_utils::requireInput<double>(*this, PortKeys::kPositionTolerance);
    const double ori_tol =
        bt_utils::requireInput<double>(*this, PortKeys::kOrientationTolerance);
    const double joint_tol =
        bt_utils::requireInput<double>(*this, PortKeys::kJointTolerance);

    // Prefer pose goal if present.
    const auto pose_in = getInput<geometry_msgs::msg::PoseStamped>(PortKeys::kTargetPose);
    if (pose_in)
    {
      const auto link_name =
          bt_utils::requireInput<std::string>(*this, PortKeys::kLinkName);
      const auto constraints =
          makePoseGoalConstraints(pose_in.value(), link_name, pos_tol, ori_tol);

      req->goal_constraints.clear();
      req->goal_constraints.push_back(constraints);

      const std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest> out = req;
      bt_utils::setOutput(*this, PortKeys::kRequest, out);
      bt_utils::setOutput(*this, PortKeys::kError, std::string());

      RCLCPP_INFO(logger_, "built pose request group='%s' link='%s'",
                  req->group_name.c_str(), link_name.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    // Otherwise joint goal.
    const auto joint_names =
        bt_utils::requireInput<std::vector<std::string>>(*this, PortKeys::kJointNames);
    const auto joint_positions =
        bt_utils::requireInput<std::vector<double>>(*this, PortKeys::kJointPositions);

    if (joint_names.empty())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: '",
                             PortKeys::kJointNames, "' is empty");
    }

    if (joint_names.size() != joint_positions.size())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: joint_names.size != joint_positions.size");
    }

    const auto constraints =
        makeJointGoalConstraints(joint_names, joint_positions, joint_tol);

    req->goal_constraints.clear();
    req->goal_constraints.push_back(constraints);

    const std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest> out = req;
    bt_utils::setOutput(*this, PortKeys::kRequest, out);
    bt_utils::setOutput(*this, PortKeys::kError, std::string());

    RCLCPP_INFO(logger_, "built joint request group='%s' joints=%zu",
                req->group_name.c_str(), joint_names.size());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus BuildMotionPlanRequest::failWithError(const std::string& msg)
{
  bt_utils::setOutput(*this, PortKeys::kRequest,
                      std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest>());
  bt_utils::setOutput(*this, PortKeys::kError, msg);
  RCLCPP_ERROR(logger_, "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}
}  // namespace rlc_executive
