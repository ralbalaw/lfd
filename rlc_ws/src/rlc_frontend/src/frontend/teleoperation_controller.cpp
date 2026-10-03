#include "rlc_frontend/frontend/teleoperation_controller.hpp"

#include <memory>
#include <stdexcept>

#include "rlc_frontend/frontend/servo_joy_frontend.hpp"

namespace rlc_frontend
{
namespace
{

std::unique_ptr<TeleoperationFrontend> makeFrontend(rclcpp::Node* node,
                                                    const TeleoperationConfig& cfg,
                                                    TeleoperationSession& session)
{
  if (cfg.frontend == "joy")
  {
    return std::make_unique<ServoJoyFrontend>(node, cfg.joy, session);
  }

  throw std::invalid_argument(
      "TeleoperationController: unsupported teleoperation frontend '" + cfg.frontend +
      "'");
}

}  // namespace

TeleoperationController::TeleoperationController(rclcpp::Node* node,
                                                 const TeleoperationConfig& teleop_cfg)
  : cfg_(teleop_cfg)
  , servo_client_{ node, &cfg_.servo }
  , session_{ node }
  , frontend_{ makeFrontend(node, cfg_, session_) }
{ frontend_->setEnabled(false); }

ServoPauseResult TeleoperationController::setActive(bool active,
                                                    std::chrono::milliseconds timeout)
{
  if (!active)
  {
    frontend_->setEnabled(false);
    active_.store(false, std::memory_order_relaxed);
    session_.setRequest(DemonstrationRequest::kNone);
    return servo_client_.setPaused(true, timeout);
  }

  const ServoCommandTypeResult command_type_result =
      servo_client_.setCommandType(MoveItServoClient::CommandType::kTwist, timeout);
  if (!command_type_result.success)
  {
    frontend_->setEnabled(false);
    active_.store(false, std::memory_order_relaxed);

    ServoPauseResult result;
    result.paused = true;
    result.success = false;
    result.message = "failed to select Servo twist command type: " +
                     command_type_result.message;
    return result;
  }

  const ServoPauseResult result = servo_client_.setPaused(false, timeout);
  if (!result.success)
  {
    frontend_->setEnabled(false);
    active_.store(false, std::memory_order_relaxed);
    return result;
  }

  session_.setRequest(DemonstrationRequest::kNone);
  frontend_->setEnabled(true);
  active_.store(true, std::memory_order_relaxed);
  return result;
}

DemonstrationRequest TeleoperationController::takeRequest()
{ return session_.takeRequest(); }

DemonstrationRequest TeleoperationController::peekRequest() const
{ return session_.peekRequest(); }

std::optional<ServoStatusSnapshot> TeleoperationController::status() const
{ return servo_client_.status(); }

}  // namespace rlc_frontend
