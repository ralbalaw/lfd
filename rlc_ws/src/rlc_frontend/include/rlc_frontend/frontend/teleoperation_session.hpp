#pragma once

#include <mutex>
#include <optional>

#include <rclcpp/node.hpp>

#include "rlc_frontend/frontend/types.hpp"

namespace rlc_frontend
{

/**
 * @brief Stores the latest operator request produced by the active teleoperation frontend.
 */
class TeleoperationSession final
{
public:
  /**
   * @brief Creates a teleoperation request latch.
   * @param[in] node ROS node used to timestamp incoming requests.
   */
  explicit TeleoperationSession(rclcpp::Node* node);

  /**
   * @brief Returns and clears the currently latched operator request.
   * @return Pending operator request, or `DemonstrationRequest::kNone` if no request is pending.
   */
  DemonstrationRequest takeRequest();

  /**
   * @brief Returns the currently latched operator request without clearing it.
   * @return Pending operator request, or `DemonstrationRequest::kNone` if no request is pending.
   */
  DemonstrationRequest peekRequest() const;

  /**
   * @brief Stores one operator request for later consumption.
   * @param[in] request Pending operator request to latch.
   */
  void setRequest(DemonstrationRequest request);

private:
  rclcpp::Node* node_ = nullptr;

  mutable std::mutex request_mtx_;
  DemonstrationRequest request_ = DemonstrationRequest::kNone;
  std::optional<rclcpp::Time> request_stamp_;
};

}  // namespace rlc_frontend
