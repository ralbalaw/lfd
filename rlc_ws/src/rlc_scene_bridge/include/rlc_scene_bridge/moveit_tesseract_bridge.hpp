#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_environment/fwd.h>

#include <rlc_scene_bridge/scene_bridge_options.hpp>
#include <rlc_scene_bridge/tesseract_env_sync.hpp>

namespace rlc_scene_bridge
{

class MoveItTesseractBridge
{
public:
  using PlanningSceneMsg = moveit_msgs::msg::PlanningScene;
  using PlanningSceneMsgConstPtr = PlanningSceneMsg::ConstSharedPtr;
  using PlanningSceneComponentsMsg = moveit_msgs::msg::PlanningSceneComponents;
  using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
  using MonitorInterfaceConstPtr =
      std::shared_ptr<const tesseract_environment::EnvironmentMonitorInterface>;

  MoveItTesseractBridge(rclcpp::Node::SharedPtr node,
                        MonitorInterfaceConstPtr monitor_interface,
                        MoveItTesseractBridgeOptions opt = {});

  ~MoveItTesseractBridge();

  MoveItTesseractBridge(const MoveItTesseractBridge&) = delete;
  MoveItTesseractBridge& operator=(const MoveItTesseractBridge&) = delete;

  bool isSynchronized() const noexcept;

  rclcpp::Time lastUpdateStamp() const noexcept;

  std::shared_ptr<tesseract_environment::Environment> envSnapshot() const;

  void requestSync();

  const MoveItTesseractBridgeOptions& options() const noexcept
  {
    return opt_;
  }

  rclcpp::CallbackGroup::SharedPtr callbackGroup() const
  {
    return cbg_;
  }

private:
  enum class EnvState : uint8_t
  {
    UNINITIALIZED,
    SYNCHRONIZING,
    SYNCHRONIZED
  };

  void onSceneUpdate(const PlanningSceneMsgConstPtr& msg);

  void onSyncResponse(const rclcpp::Client<GetPlanningScene>::SharedFuture& future);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  MoveItTesseractBridgeOptions opt_;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Subscription<PlanningSceneMsg>::SharedPtr scene_sub_;
  rclcpp::Client<GetPlanningScene>::SharedPtr get_scene_client_;

  std::atomic<EnvState> env_state_{ EnvState::UNINITIALIZED };
  std::atomic<int64_t> last_update_ns_{ 0 };

  std::unique_ptr<TesseractEnvSync> env_sync_;

  mutable std::mutex sync_mtx_;
};

}  // namespace rlc_scene_bridge
