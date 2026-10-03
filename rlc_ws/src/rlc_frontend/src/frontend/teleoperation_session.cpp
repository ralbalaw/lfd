#include "rlc_frontend/frontend/teleoperation_session.hpp"

namespace rlc_frontend
{

TeleoperationSession::TeleoperationSession(rclcpp::Node* node) : node_(node)
{
}

DemonstrationRequest TeleoperationSession::takeRequest()
{
  std::scoped_lock lock(request_mtx_);

  const DemonstrationRequest out = request_;
  request_ = DemonstrationRequest::kNone;
  request_stamp_.reset();
  return out;
}

DemonstrationRequest TeleoperationSession::peekRequest() const
{
  std::scoped_lock lock(request_mtx_);
  return request_;
}

void TeleoperationSession::setRequest(DemonstrationRequest request)
{
  std::scoped_lock lock(request_mtx_);
  request_ = request;

  if (request == DemonstrationRequest::kNone)
  {
    request_stamp_.reset();
    return;
  }

  request_stamp_ = node_->now();
}

}  // namespace rlc_frontend
