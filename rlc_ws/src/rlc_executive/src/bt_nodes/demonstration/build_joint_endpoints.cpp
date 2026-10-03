#include "rlc_executive/bt_nodes/demonstration/build_joint_endpoints.hpp"

#include <stdexcept>

#include <Eigen/Core>
#include <behaviortree_cpp/exceptions.h>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

BuildJointEndpoints::BuildJointEndpoints(const std::string& name,
                                         const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus BuildJointEndpoints::tick()
{
  const auto trajectory = bt_utils::requireInput<std::shared_ptr<const JointTrajectory>>(
      *this, PortKeys::kTrajectory);
  if (!trajectory)
  {
    throw BT::RuntimeError(registrationName(), "[", fullPath(),
                           "]: input trajectory is null");
  }
  if (trajectory->length() == 0U)
  {
    throw std::runtime_error("trajectory is empty");
  }

  const Eigen::Index first = 0;
  const Eigen::Index last = static_cast<Eigen::Index>(trajectory->length() - 1U);

  JointEndpoints endpoints;
  endpoints.start = trajectory->positions().row(last).transpose();
  endpoints.goal = trajectory->positions().row(first).transpose();

  bt_utils::setOutput(*this, PortKeys::kJointEndpoints, endpoints);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rlc_executive
