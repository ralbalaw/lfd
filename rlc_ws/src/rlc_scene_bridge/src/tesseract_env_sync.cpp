#include <stdexcept>
#include <utility>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_monitor_interface.h>
#include <rlc_scene_bridge/tesseract_env_sync.hpp>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/remove_link_command.h>
#include <tesseract_environment/commands/change_joint_origin_command.h>
#include <Eigen/Geometry>

namespace rlc_scene_bridge
{

TesseractEnvSync::TesseractEnvSync(MonitorInterfaceConstPtr monitor_interface,
                                   std::string monitor_namespace,
                                   TesseractEnvSyncOptions opt)
  : monitor_interface_(std::move(monitor_interface))
  , monitor_namespace_(std::move(monitor_namespace))
  , opt_(std::move(opt))
{
  if (!monitor_interface_)
  {
    throw std::invalid_argument("TesseractEnvSync: monitor_interface is null");
  }

  if (monitor_namespace_.empty())
  {
    throw std::invalid_argument("TesseractEnvSync: monitor_namespace is empty");
  }
}

TesseractEnvSync::~TesseractEnvSync() = default;

TesseractEnvSync::EnvironmentUPtr TesseractEnvSync::snapshot() const
{
  std::scoped_lock lk(monitor_mutex_);
  auto env = monitor_interface_->getEnvironment(monitor_namespace_);
  if (!env)
  {
    throw std::runtime_error("TesseractEnvSync: monitor returned null environment for "
                             "namespace '" +
                             monitor_namespace_ + "'");
  }

  return env;
}

void TesseractEnvSync::applyFullScene(const PlanningSceneMsg& scene)
{
  auto commands = fromFullScene(scene);
  applyCommands(commands);
}

void TesseractEnvSync::applyDiff(const PlanningSceneMsg& scene)
{
  auto commands = fromDiff(scene);
  applyCommands(commands);
}

void TesseractEnvSync::applyCommands(const tesseract_environment::Commands& commands) const
{
  std::scoped_lock lk(monitor_mutex_);
  const bool ok = monitor_interface_->applyCommands(monitor_namespace_, commands);
  if (!ok)
  {
    throw std::runtime_error("TesseractEnvSync: applyCommands failed for namespace '" +
                             monitor_namespace_ + "'");
  }
}

tesseract_environment::Commands TesseractEnvSync::fromFullScene(
    const PlanningSceneMsg& scene)
{
  tesseract_environment::Commands commands;
  commands.reserve(world_objects_.size() + attached_objects_.size() +
                   scene.world.collision_objects.size() +
                   scene.robot_state.attached_collision_objects.size());

  // Clear existing world objects.
  for (const auto& kv : world_objects_)
  {
    const auto& info = kv.second;
    commands.push_back(
        std::make_shared<tesseract_environment::RemoveLinkCommand>(info.link_name));
  }

  // Clear existing attached objects.
  for (const auto& kv : attached_objects_)
  {
    const auto& info = kv.second;
    commands.push_back(
        std::make_shared<tesseract_environment::RemoveLinkCommand>(info.link_name));
  }

  world_objects_.clear();
  attached_objects_.clear();

  // Add all world objects.
  for (const auto& object : scene.world.collision_objects)
  {
    auto cmd = makeAddWorldCommand(object);
    commands.push_back(std::move(cmd));
  }

  // Add all attached objects.
  for (const auto& aco : scene.robot_state.attached_collision_objects)
  {
    auto cmd = makeAddAttachedCommand(aco);
    commands.push_back(std::move(cmd));
  }

  return commands;
}

tesseract_environment::Commands TesseractEnvSync::fromDiff(
    const PlanningSceneMsg& scene)
{
  tesseract_environment::Commands commands;
  commands.reserve(scene.world.collision_objects.size() +
                   scene.robot_state.attached_collision_objects.size() * 2);

  // 1) World diffs.
  for (const auto& object : scene.world.collision_objects)
  {
    tesseract_environment::Command::Ptr cmd{};

    switch (object.operation)
    {
      case moveit_msgs::msg::CollisionObject::ADD:
      {
        cmd = makeAddWorldCommand(object);
        break;
      }

      case moveit_msgs::msg::CollisionObject::REMOVE:
      {
        cmd = makeRemoveWorldCommand(object);
        break;
      }

      case moveit_msgs::msg::CollisionObject::MOVE:
      {
        cmd = makeMoveWorldCommand(object);
        break;
      }

      case moveit_msgs::msg::CollisionObject::APPEND:
      {
        throw std::domain_error("TesseractEnvSync: APPEND unsupported for world objects "
                                "(id='" +
                                object.id + "')");
      }

      default:
      {
        throw std::domain_error("TesseractEnvSync: unsupported world operation " +
                                std::to_string(static_cast<int>(object.operation)) +
                                " (id='" + object.id + "')");
      }
    }

    commands.push_back(std::move(cmd));
  }

  // 2) Attached diffs.
  for (const auto& aco : scene.robot_state.attached_collision_objects)
  {
    switch (aco.object.operation)
    {
      case moveit_msgs::msg::CollisionObject::ADD:
      {
        commands.push_back(makeAddAttachedCommand(aco));
        break;
      }

      case moveit_msgs::msg::CollisionObject::REMOVE:
      {
        appendRemoveAttachedCommands(aco, commands);
        break;
      }

      case moveit_msgs::msg::CollisionObject::MOVE:
      {
        throw std::domain_error("TesseractEnvSync: MOVE unsupported for attached objects "
                                "(id='" +
                                aco.object.id + "')");
      }

      default:
      {
        throw std::domain_error(
            "TesseractEnvSync: unsupported attached operation " +
            std::to_string(static_cast<unsigned>(aco.object.operation)) + " (id='" +
            aco.object.id + "')");
      }
    }
  }

  return commands;
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeAddWorldCommand(
    const moveit_msgs::msg::CollisionObject& object)
{
  const auto built = tesseract_scene::buildWorldObject(object, opt_.naming_policy);
  world_objects_.insert_or_assign(built.info.id, built.info);

  return std::make_shared<tesseract_environment::AddLinkCommand>(
      built.link, built.joint, opt_.allow_replace);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeAddAttachedCommand(
    const moveit_msgs::msg::AttachedCollisionObject& aco)
{
  const auto built = tesseract_scene::buildAttachedObject(aco, opt_.naming_policy);
  attached_objects_.insert_or_assign(built.info.id, built.info);

  return std::make_shared<tesseract_environment::AddLinkCommand>(
      built.link, built.joint, opt_.allow_replace);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeMoveWorldCommand(
    const moveit_msgs::msg::CollisionObject& object) const
{
  const std::string& id = object.id;
  if (id.empty())
  {
    throw std::domain_error("TesseractEnvSync: MOVE world object with empty id");
  }

  auto it = world_objects_.find(id);
  if (it == world_objects_.end())
  {
    throw std::domain_error("TesseractEnvSync: MOVE for unknown world object '" + id +
                            "'");
  }

  const tesseract_scene::ObjectInfo& info = it->second;

  // v1: reject frame changes (requires TF resolution / parent relinking)
  if (!object.header.frame_id.empty() && object.header.frame_id != info.frame_id)
  {
    throw std::domain_error("TesseractEnvSync: MOVE for '" + id +
                            "' changed frame_id from '" + info.frame_id + "' to '" +
                            object.header.frame_id + "'");
  }

  const Eigen::Isometry3d origin = tesseract_scene::poseToIsometry(object.pose);

  return std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      info.joint_name, origin);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeRemoveWorldCommand(
    const moveit_msgs::msg::CollisionObject& object)
{
  const std::string& id = object.id;
  if (id.empty())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE world object with empty id");
  }

  auto it = world_objects_.find(id);
  if (it == world_objects_.end())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE for unknown world object '" + id +
                            "'");
  }

  const std::string link_name = it->second.link_name;
  world_objects_.erase(it);

  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeRemoveAttachedCommand(
    const moveit_msgs::msg::CollisionObject& object)
{
  const std::string& id = object.id;
  if (id.empty())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE attached object with empty id");
  }

  auto it = attached_objects_.find(id);
  if (it == attached_objects_.end())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE for unknown attached object '" +
                            id + "'");
  }

  const std::string link_name = it->second.link_name;
  attached_objects_.erase(it);

  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

void TesseractEnvSync::appendRemoveAttachedCommands(
    const moveit_msgs::msg::AttachedCollisionObject& aco,
    tesseract_environment::Commands& commands)
{
  if (aco.link_name.empty())
  {
    throw std::domain_error(
        "TesseractEnvSync: REMOVE attached objects with empty link_name");
  }

  const std::string& id = aco.object.id;

  if (id.empty())
  {
    for (auto it = attached_objects_.begin(); it != attached_objects_.end();)
    {
      if (it->second.parent_link == aco.link_name)
      {
        commands.push_back(std::make_shared<tesseract_environment::RemoveLinkCommand>(
            it->second.link_name));
        it = attached_objects_.erase(it);
      }
      else
      {
        ++it;
      }
    }
    return;
  }

  // Remove single by id
  auto cmd = makeRemoveAttachedCommand(aco.object);

  commands.push_back(std::move(cmd));
}

}  // namespace rlc_scene_bridge
