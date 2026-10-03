#pragma once

#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>

namespace rlc_scene_bridge
{
namespace tesseract_scene
{

struct NamingPolicy
{
  std::string world_link_prefix{ "w_" };
  std::string attached_link_prefix{ "a_" };
};
struct ObjectInfo
{
  std::string id;
  std::string frame_id;
  std::string link_name;
  std::string joint_name;
  std::string parent_link;
};
struct BuiltObject
{
  ObjectInfo info;
  tesseract_scene_graph::Link link;
  tesseract_scene_graph::Joint joint;
};

std::string worldLinkName(const NamingPolicy& naming_policy, const std::string& id);
std::string attachedLinkName(const NamingPolicy& naming_policy, const std::string& id);
std::string jointName(const std::string& link_name);

/**
 * @brief Build a fixed-joint world object for insertion into a Tesseract scene graph.
 * @param[in] object MoveIt collision object. Only primitives and planes are supported.
 * @param[in] naming_policy Naming policy used to derive link and joint names.
 * @return Built object containing resolved link/joint and metadata.
 * @throws std::domain_error If the input message is invalid or uses unsupported
 * geometry (e.g., meshes).
 */
BuiltObject buildWorldObject(const moveit_msgs::msg::CollisionObject& object,
                             const NamingPolicy& naming_policy);

/**
 * @brief Build a fixed-joint attached object for insertion into a Tesseract scene graph.
 * @param[in] aco MoveIt attached collision object. Only primitives and planes are supported.
 * @param[in] naming_policy Naming policy used to derive link and joint names.
 * @return Built object containing resolved link/joint and metadata.
 * @throws std::domain_error If the input message is invalid or uses unsupported
 * geometry (e.g., meshes) or is missing the parent link name.
 */
BuiltObject buildAttachedObject(const moveit_msgs::msg::AttachedCollisionObject& aco,
                                const NamingPolicy& naming_policy);
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose);
}  // namespace tesseract_scene
}  // namespace rlc_scene_bridge
