#include "rlc_frontend/frontend/moveit_servo_client.hpp"

#include <future>
#include <stdexcept>

namespace rlc_frontend
{

MoveItServoClient::MoveItServoClient(rclcpp::Node* node, const ServoConfig* cfg)
  : node_(node), cfg_(cfg)
{
  if (cfg_->pause_service_name.empty())
  {
    throw std::invalid_argument(
        "MoveItServoClient: pause_service_name must not be empty");
  }
  if (cfg_->switch_command_type_service_name.empty())
  {
    throw std::invalid_argument(
        "MoveItServoClient: switch_command_type_service_name must not be empty");
  }
  if (cfg_->status_topic.empty())
  {
    throw std::invalid_argument("MoveItServoClient: status_topic must not be empty");
  }

  pause_client_ = node_->create_client<PauseSrv>(cfg_->pause_service_name);
  cmd_type_client_ =
      node_->create_client<CmdTypeSrv>(cfg_->switch_command_type_service_name);

  status_sub_ = node_->create_subscription<moveit_msgs::msg::ServoStatus>(
      cfg_->status_topic, rclcpp::QoS(10),
      [this](const moveit_msgs::msg::ServoStatus::ConstSharedPtr& msg) { onStatus(msg); });
}

ServoPauseResult MoveItServoClient::setPaused(bool paused,
                                              std::chrono::milliseconds timeout)
{
  ServoPauseResult out;
  out.paused = paused;

  if (!pause_client_)
  {
    out.success = false;
    out.message = "pause_servo client is null";
    return out;
  }

  if (!pause_client_->wait_for_service(timeout))
  {
    out.success = false;
    out.message = "pause_servo service not available: " + cfg_->pause_service_name;
    return out;
  }

  auto req = std::make_shared<PauseSrv::Request>();
  req->data = paused;

  auto fut = pause_client_->async_send_request(req);

  if (fut.wait_for(timeout) != std::future_status::ready)
  {
    out.success = false;
    out.message = "pause_servo call timed out";
    return out;
  }

  const auto res = fut.get();
  out.success = res->success;
  out.message = res->message;
  return out;
}

ServoCommandTypeResult
MoveItServoClient::setCommandType(CommandType type, std::chrono::milliseconds timeout)
{
  ServoCommandTypeResult out;
  out.command_type = static_cast<std::int8_t>(type);

  if (!cmd_type_client_)
  {
    out.success = false;
    out.message = "switch_command_type client is null";
    return out;
  }

  if (!cmd_type_client_->wait_for_service(timeout))
  {
    out.success = false;
    out.message = "switch_command_type service not available: " +
                  cfg_->switch_command_type_service_name;
    return out;
  }

  auto req = std::make_shared<CmdTypeSrv::Request>();
  req->command_type = static_cast<std::int8_t>(type);

  auto fut = cmd_type_client_->async_send_request(req);

  if (fut.wait_for(timeout) != std::future_status::ready)
  {
    out.success = false;
    out.message = "switch_command_type call timed out";
    return out;
  }

  const auto res = fut.get();
  out.success = res->success;
  out.message = res->success ? "" : "switch_command_type rejected request";
  return out;
}

std::optional<ServoStatusSnapshot> MoveItServoClient::status() const
{
  std::lock_guard<std::mutex> lock(status_mtx_);
  if (!last_status_)
  {
    return std::nullopt;
  }

  ServoStatusSnapshot snap;
  snap.status = *last_status_;

  const auto now = node_->now();
  const auto age = (now - last_status_stamp_).seconds();
  snap.age_sec = age;
  return snap;
}

void MoveItServoClient::onStatus(const moveit_msgs::msg::ServoStatus::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(status_mtx_);
  last_status_ = *msg;
  last_status_stamp_ = node_->now();
}

}  // namespace rlc_frontend