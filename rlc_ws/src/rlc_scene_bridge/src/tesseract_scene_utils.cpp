#include "rlc_scene_bridge/tesseract_scene_utils.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>
#include <shape_msgs/msg/plane.hpp>

namespace rlc_scene_bridge
{
namespace tesseract_scene
{
namespace
{

std::shared_ptr<tesseract_geometry::Geometry>
makePrimitiveGeometry(const shape_msgs::msg::SolidPrimitive& prim)
{
  using shape_msgs::msg::SolidPrimitive;
  switch (prim.type)
  {
    case SolidPrimitive::BOX:
    {
      if (prim.dimensions.size() < 3U)
      {
        return nullptr;
      }
      const double x = prim.dimensions[SolidPrimitive::BOX_X];
      const double y = prim.dimensions[SolidPrimitive::BOX_Y];
      const double z = prim.dimensions[SolidPrimitive::BOX_Z];
      return std::make_shared<tesseract_geometry::Box>(x, y, z);
    }

    case SolidPrimitive::SPHERE:
    {
      if (prim.dimensions.size() < 1U)
      {
        return nullptr;
      }
      const double r = prim.dimensions[SolidPrimitive::SPHERE_RADIUS];
      return std::make_shared<tesseract_geometry::Sphere>(r);
    }

    case SolidPrimitive::CYLINDER:
    {
      if (prim.dimensions.size() < 2U)
      {
        return nullptr;
      }
      const double h = prim.dimensions[SolidPrimitive::CYLINDER_HEIGHT];
      const double r = prim.dimensions[SolidPrimitive::CYLINDER_RADIUS];
      return std::make_shared<tesseract_geometry::Cylinder>(r, h);
    }

    case SolidPrimitive::CONE:
    {
      if (prim.dimensions.size() < 2U)
      {
        return nullptr;
      }
      const double h = prim.dimensions[SolidPrimitive::CONE_HEIGHT];
      const double r = prim.dimensions[SolidPrimitive::CONE_RADIUS];
      return std::make_shared<tesseract_geometry::Cone>(r, h);
    }

    default:
      return nullptr;
  }
}

std::shared_ptr<tesseract_geometry::Geometry>
makePlaneGeometry(const shape_msgs::msg::Plane& plane)
{
  const double a = plane.coef[0];
  const double b = plane.coef[1];
  const double c = plane.coef[2];
  const double d = plane.coef[3];

  const double n2 = a * a + b * b + c * c;
  if (n2 <= 0.0)
  {
    return nullptr;
  }

  const double inv_n = 1.0 / std::sqrt(n2);

  return std::make_shared<tesseract_geometry::Plane>(a * inv_n, b * inv_n, c * inv_n,
                                                     d * inv_n);
}

std::shared_ptr<tesseract_scene_graph::Visual>
makeVisual(const std::string& name, const Eigen::Isometry3d& origin,
           const std::shared_ptr<tesseract_geometry::Geometry>& geometry)
{
  auto visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->name = name;
  visual->origin = origin;
  visual->geometry = geometry;
  return visual;
}

std::shared_ptr<tesseract_scene_graph::Collision>
makeCollision(const std::string& name, const Eigen::Isometry3d& origin,
              const std::shared_ptr<tesseract_geometry::Geometry>& geometry)
{
  auto collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->name = name;
  collision->origin = origin;
  collision->geometry = geometry;
  return collision;
}

/**
 * @brief Validate a MoveIt collision object prior to conversion to a fixed-joint
 * Tesseract link.
 * @param[in] object Collision object to validate.
 * @throws std::domain_error If the input is invalid or uses unsupported geometry.
 */
void validateCollisionObject(const moveit_msgs::msg::CollisionObject& object)
{
  if (object.id.empty())
  {
    throw std::domain_error("CollisionObject.id is empty");
  }
  if (!object.meshes.empty())
  {
    throw std::domain_error("CollisionObject '" + object.id +
                            "' has meshes; meshes are not supported");
  }
  if (object.header.frame_id.empty())
  {
    throw std::domain_error("CollisionObject '" + object.id +
                            "' has empty header.frame_id");
  }
  if (object.primitives.size() != object.primitive_poses.size())
  {
    throw std::domain_error("CollisionObject '" + object.id +
                            "' primitives.size() != primitive_poses.size()");
  }
  if (object.planes.size() != object.plane_poses.size())
  {
    throw std::domain_error("CollisionObject '" + object.id +
                            "' planes.size() != plane_poses.size()");
  }
  if (object.primitives.empty() && object.planes.empty())
  {
    throw std::domain_error("CollisionObject '" + object.id +
                            "' has no primitives or planes");
  }
}

/**
 * @brief Append geometry from a MoveIt collision object to a Tesseract link.
 * @param[in] object Collision object containing primitive/plane geometry.
 * @param[in] link_name Name used to generate per-shape visual/collision names.
 * @param[in,out] link Link to append visual and collision elements to.
 * @throws std::domain_error If any primitive or plane is invalid or unsupported.
 */
void fillLinkGeometry(const moveit_msgs::msg::CollisionObject& object,
                      const std::string& link_name,
                      tesseract_scene_graph::Link& link)
{
  for (std::size_t i = 0; i < object.primitives.size(); ++i)
  {
    const auto geometry = makePrimitiveGeometry(object.primitives[i]);
    if (!geometry)
    {
      throw std::domain_error("CollisionObject '" + object.id +
                              "' has invalid primitive at index " +
                              std::to_string(i));
    }
    const Eigen::Isometry3d origin = poseToIsometry(object.primitive_poses[i]);
    const auto visual_name = link_name + "_v_" + std::to_string(i);
    const auto collision_name = link_name + "_c_" + std::to_string(i);
    link.visual.push_back(makeVisual(visual_name, origin, geometry));
    link.collision.push_back(makeCollision(collision_name, origin, geometry));
  }

  for (std::size_t i = 0; i < object.planes.size(); ++i)
  {
    const auto geometry = makePlaneGeometry(object.planes[i]);
    if (!geometry)
    {
      throw std::domain_error("CollisionObject '" + object.id +
                              "' has invalid plane at index " +
                              std::to_string(i));
    }
    const Eigen::Isometry3d origin = poseToIsometry(object.plane_poses[i]);
    const auto visual_name = link_name + "_v_plane_" + std::to_string(i);
    const auto collision_name = link_name + "_c_plane_" + std::to_string(i);
    link.visual.push_back(makeVisual(visual_name, origin, geometry));
    link.collision.push_back(makeCollision(collision_name, origin, geometry));
  }
}

BuiltObject buildFixedLinkObject(const moveit_msgs::msg::CollisionObject& object,
                                 const std::string& parent_link,
                                 const std::string& link_name,
                                 const std::string& joint_name)
{
  tesseract_scene_graph::Link link(link_name);
  fillLinkGeometry(object, link_name, link);

  tesseract_scene_graph::Joint joint(joint_name);
  joint.parent_link_name = parent_link;
  joint.child_link_name = link_name;
  joint.type = tesseract_scene_graph::JointType::FIXED;
  joint.parent_to_joint_origin_transform = poseToIsometry(object.pose);

  ObjectInfo info;
  info.id = object.id;
  info.frame_id = object.header.frame_id;
  info.link_name = link_name;
  info.joint_name = joint_name;
  info.parent_link = joint.parent_link_name;

  return BuiltObject{ std::move(info), std::move(link), std::move(joint) };
}

}  // namespace

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

  const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                             pose.orientation.z);

  if (q.norm() > 0.0)
  {
    out.linear() = q.normalized().toRotationMatrix();
  }

  return out;
}

std::string worldLinkName(const NamingPolicy& naming_policy, const std::string& id)
{ return naming_policy.world_link_prefix + id; }

std::string attachedLinkName(const NamingPolicy& naming_policy, const std::string& id)
{ return naming_policy.attached_link_prefix + id; }

std::string jointName(const std::string& link_name)
{ return "joint_" + link_name; }

BuiltObject buildWorldObject(const moveit_msgs::msg::CollisionObject& object,
                             const NamingPolicy& naming_policy)
{
  validateCollisionObject(object);
  const auto& parent_frame = object.header.frame_id;

  const std::string link_name = worldLinkName(naming_policy, object.id);
  const std::string joint_name = jointName(link_name);

  return buildFixedLinkObject(object, parent_frame, link_name, joint_name);
}

BuiltObject buildAttachedObject(const moveit_msgs::msg::AttachedCollisionObject& aco,
                                const NamingPolicy& naming_policy)
{
  const auto& object = aco.object;

  validateCollisionObject(object);

  const auto& parent_frame = aco.link_name;

  if (parent_frame.empty())
  {
    throw std::domain_error("AttachedCollisionObject has empty link_name for object '" +
                            object.id + "'");
  }

  const std::string link_name = attachedLinkName(naming_policy, object.id);
  const std::string joint_name = jointName(link_name);

  return buildFixedLinkObject(object, parent_frame, link_name, joint_name);
}

}  // namespace tesseract_scene
}  // namespace rlc_scene_bridge
