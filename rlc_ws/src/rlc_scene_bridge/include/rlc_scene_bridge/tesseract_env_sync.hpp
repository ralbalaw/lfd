#pragma once
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>
#include <tesseract_environment/fwd.h>
#include <tesseract_environment/commands.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rlc_scene_bridge/scene_bridge_options.hpp>
#include <rlc_scene_bridge/tesseract_scene_utils.hpp>

namespace rlc_scene_bridge
{

class TesseractEnvSync
{
public:
  using PlanningSceneMsg = moveit_msgs::msg::PlanningScene;
  using MonitorInterfaceConstPtr =
      std::shared_ptr<const tesseract_environment::EnvironmentMonitorInterface>;
  using EnvironmentUPtr = std::unique_ptr<tesseract_environment::Environment>;

  TesseractEnvSync(MonitorInterfaceConstPtr monitor_interface,
                   std::string monitor_namespace, TesseractEnvSyncOptions opt = {});

  ~TesseractEnvSync();

  TesseractEnvSync(const TesseractEnvSync&) = delete;
  TesseractEnvSync& operator=(const TesseractEnvSync&) = delete;

  void applyFullScene(const PlanningSceneMsg& scene);

  void applyDiff(const PlanningSceneMsg& scene);

  EnvironmentUPtr snapshot() const;

  const std::string& monitorNamespace() const noexcept
  {
    return monitor_namespace_;
  }

  const TesseractEnvSyncOptions& options() const noexcept
  {
    return opt_;
  }

private:
  tesseract_environment::Commands fromFullScene(const PlanningSceneMsg& scene);

  tesseract_environment::Commands fromDiff(const PlanningSceneMsg& scene);

  void applyCommands(const tesseract_environment::Commands& commands) const;

  tesseract_environment::Command::Ptr
  makeAddWorldCommand(const moveit_msgs::msg::CollisionObject& object);

  tesseract_environment::Command::Ptr
  makeRemoveWorldCommand(const moveit_msgs::msg::CollisionObject& object);

  tesseract_environment::Command::Ptr
  makeMoveWorldCommand(const moveit_msgs::msg::CollisionObject& object) const;

  tesseract_environment::Command::Ptr
  makeAddAttachedCommand(const moveit_msgs::msg::AttachedCollisionObject& aco);

  tesseract_environment::Command::Ptr
  makeRemoveAttachedCommand(const moveit_msgs::msg::CollisionObject& object);

  void appendRemoveAttachedCommands(const moveit_msgs::msg::AttachedCollisionObject& aco,
                                    tesseract_environment::Commands& commands);

  MonitorInterfaceConstPtr monitor_interface_;
  std::string monitor_namespace_;
  TesseractEnvSyncOptions opt_;

  std::unordered_map<std::string, tesseract_scene::ObjectInfo> world_objects_;
  std::unordered_map<std::string, tesseract_scene::ObjectInfo> attached_objects_;

  mutable std::mutex monitor_mutex_;
};

}  // namespace rlc_scene_bridge
