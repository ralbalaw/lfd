#include <rlc_scene_bridge/moveit_tesseract_bridge.hpp>

#include <stdexcept>
#include <typeinfo>
#include <utility>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_environment/environment.h>

namespace rlc_scene_bridge
{
MoveItTesseractBridge::MoveItTesseractBridge(rclcpp::Node::SharedPtr node,
                                             MonitorInterfaceConstPtr monitor_interface,
                                             MoveItTesseractBridgeOptions opt)
  : node_(std::move(node))
  , logger_(rclcpp::get_logger("rlc_scene_bridge.scene_bridge"))
  , opt_(std::move(opt))
{
  if (!node_)
  {
    throw std::invalid_argument("node is null");
  }

  logger_ = node_->get_logger().get_child("rlc_scene_bridge.scene_bridge");

  env_sync_ = std::make_unique<TesseractEnvSync>(std::move(monitor_interface),
                                                 opt_.monitor_namespace, opt_.env_sync);

  cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::QoS qos(rclcpp::KeepLast(opt_.scene_qos_depth));
  qos.reliable();
  qos.durability_volatile();

  rclcpp::SubscriptionOptions sub_opts;

  sub_opts.event_callbacks.incompatible_qos_callback =
      [this](rclcpp::QOSRequestedIncompatibleQoSInfo& info) {
        RCLCPP_ERROR(logger_,
                     "Incompatible QoS on monitored planning scene subscription. "
                     "Last policy kind: %d, total_count: %d",
                     info.last_policy_kind, info.total_count);
      };
  sub_opts.callback_group = cbg_;

  auto scene_cb = [this](const PlanningSceneMsgConstPtr& msg) { onSceneUpdate(msg); };

  scene_sub_ = node_->create_subscription<PlanningSceneMsg>(opt_.scene_topic, qos,
                                                            scene_cb, sub_opts);

  get_scene_client_ = node_->create_client<GetPlanningScene>(opt_.get_scene_srv,
                                                             rclcpp::ServicesQoS(), cbg_);

  requestSync();
}

MoveItTesseractBridge::~MoveItTesseractBridge() = default;

void MoveItTesseractBridge::onSceneUpdate(const PlanningSceneMsgConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  const auto now_ns = node_->now().nanoseconds();

  // Full scene.
  if (!msg->is_diff)
  {
    try
    {
      env_sync_->applyFullScene(*msg);
      env_state_.store(EnvState::SYNCHRONIZED, std::memory_order_release);
      last_update_ns_.store(now_ns, std::memory_order_release);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_,
                   "MoveItTesseractBridge: rejected full PlanningScene requesting "
                   "baseline resync: (%s): %s",
                   typeid(e).name(), e.what());
      env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
      requestSync();
    }
    return;
  }

  // Diff scene.

  const auto env_state = env_state_.load(std::memory_order_acquire);

  switch (env_state)
  {
    case EnvState::UNINITIALIZED:
    {
      requestSync();
      return;
    }
    case EnvState::SYNCHRONIZING:
    {
      return;
    }
    case EnvState::SYNCHRONIZED:
    {
      try
      {
        env_sync_->applyDiff(*msg);
        last_update_ns_.store(now_ns, std::memory_order_release);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(logger_,
                     "MoveItTesseractBridge: failed to apply PlanningScene diff (%s): %s",
                     typeid(e).name(), e.what());

        env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
        requestSync();
      }
      return;
    }
    default:
      break;
  }
}

void MoveItTesseractBridge::requestSync()
{
  std::scoped_lock<std::mutex> lk(sync_mtx_);

  const auto env_state = env_state_.load(std::memory_order_acquire);
  if (env_state == EnvState::SYNCHRONIZING)
  {
    return;
  }

  if (!get_scene_client_)
  {
    RCLCPP_ERROR(logger_, "requestSync(): GetPlanningScene client is null");
    return;
  }

  if (!get_scene_client_->service_is_ready())
  {
    if (!get_scene_client_->wait_for_service(opt_.srv_wait))
    {
      RCLCPP_WARN(logger_,
                  "requestSync(): service '%s' not available (wait=%ld ms). "
                  "Will retry on next diff.",
                  opt_.get_scene_srv.c_str(), static_cast<long>(opt_.srv_wait.count()));
      return;
    }
  }

  env_state_.store(EnvState::SYNCHRONIZING, std::memory_order_release);

  auto req = std::make_shared<GetPlanningScene::Request>();
  req->components.components = opt_.scene_components;

   rclcpp::Client<GetPlanningScene>::CallbackType req_cb = [this](const rclcpp::Client<GetPlanningScene>::SharedFuture& future) {
    onSyncResponse(future);
  };

  (void)get_scene_client_->async_send_request(req, req_cb);
}

void MoveItTesseractBridge::onSyncResponse(
    const rclcpp::Client<GetPlanningScene>::SharedFuture& future)
{
  std::shared_ptr<GetPlanningScene::Response> res;

  if (env_state_.load(std::memory_order_acquire) != EnvState::SYNCHRONIZING)
  {
    RCLCPP_WARN(logger_, "onSyncResponse(): received response while not synchronizing");
    return;
  }

  try
  {
    res = future.get();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "onSyncResponse(): GetPlanningScene future.get() failed: %s",
                 e.what());
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    requestSync();
    return;
  }

  if (!res)
  {
    RCLCPP_ERROR(logger_, "onSyncResponse(): GetPlanningScene response is null");
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    requestSync();
    return;
  }

  try
  {
    env_sync_->applyFullScene(res->scene);
    env_state_.store(EnvState::SYNCHRONIZED, std::memory_order_release);
    last_update_ns_.store(node_->now().nanoseconds(), std::memory_order_release);
  }
  catch (const std::domain_error& e)
  {
    RCLCPP_WARN(logger_,
                "onSyncResponse(): rejected full PlanningScene from service; requesting "
                "baseline resync: %s",
                e.what());
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    requestSync();
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(logger_,
                 "onSyncResponse(): failed to apply full PlanningScene from service; "
                 "requesting baseline resync: %s",
                 e.what());
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    requestSync();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_,
                 "onSyncResponse(): failed to apply full PlanningScene from service; "
                 "requesting baseline resync: %s",
                 e.what());
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    requestSync();
  }
}

bool MoveItTesseractBridge::isSynchronized() const noexcept
{
  return env_state_.load(std::memory_order_relaxed) == EnvState::SYNCHRONIZED;
}

rclcpp::Time MoveItTesseractBridge::lastUpdateStamp() const noexcept
{
  const int64_t ns = last_update_ns_.load(std::memory_order_relaxed);
  return rclcpp::Time(ns, RCL_ROS_TIME);
}

std::shared_ptr<tesseract_environment::Environment>
MoveItTesseractBridge::envSnapshot() const
{
  if (!env_sync_)
  {
    RCLCPP_ERROR_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "MoveItTesseractBridge::envSnapshot(): EnvSync is not constructed.");
    return nullptr;
  }

  if (!isSynchronized())
  {
    return nullptr;
  }

  try
  {
    auto env_uptr = env_sync_->snapshot();
    return std::shared_ptr<tesseract_environment::Environment>(std::move(env_uptr));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "MoveItTesseractBridge::envSnapshot(): failed to snapshot environment: %s",
        e.what());
    return nullptr;
  }
}

}  // namespace rlc_scene_bridge
