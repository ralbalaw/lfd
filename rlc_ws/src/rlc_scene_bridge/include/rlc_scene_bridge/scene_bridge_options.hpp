#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include <moveit_msgs/msg/planning_scene_components.hpp>

#include <rlc_scene_bridge/tesseract_scene_utils.hpp>

namespace rlc_scene_bridge
{

struct TesseractEnvSyncOptions
{
  tesseract_scene::NamingPolicy naming_policy{};
  std::string world_frame{};
  bool allow_replace{ true };
};

struct MoveItTesseractBridgeOptions
{
  std::string monitor_namespace{ "tesseract_monitor" };
  std::string env_name{ "default" };
  std::string scene_topic{ "/move_group/monitored_planning_scene" };
  std::string get_scene_srv{ "/get_planning_scene" };
  uint32_t scene_components{ moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                             moveit_msgs::msg::PlanningSceneComponents::
                                 ROBOT_STATE_ATTACHED_OBJECTS |
                             moveit_msgs::msg::PlanningSceneComponents::
                                 WORLD_OBJECT_GEOMETRY };
  std::chrono::milliseconds srv_wait{ std::chrono::milliseconds{ 500 } };
  std::size_t scene_qos_depth{ 25 };
  TesseractEnvSyncOptions env_sync{};
};

}  // namespace rlc_scene_bridge

