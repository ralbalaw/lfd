#include "rlc_executive/bt_nodes/demonstration/append_trajectory.hpp"

#include <exception>
#include <memory>
#include <string>

#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

AppendTrajectory::AppendTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus AppendTrajectory::tick()
{
  try
  {
    const std::shared_ptr<const JointTrajectory> trajectory =
        bt_utils::requireInput<std::shared_ptr<const JointTrajectory>>(
            *this, PortKeys::kTrajectory);
    if (!trajectory)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: input trajectory is null");
    }

    const auto existing_demos =
        bt_utils::requireInput<std::shared_ptr<const JointTrajectories>>(
            *this, PortKeys::kDemonstrations);

    auto demonstrations = std::make_shared<JointTrajectories>();
    if (existing_demos)
    {
      *demonstrations = *existing_demos;
    }
    demonstrations->push_back(*trajectory);

    const std::shared_ptr<const JointTrajectories> out = demonstrations;
    bt_utils::setOutput(*this, PortKeys::kDemonstrations, out);
    bt_utils::setOutput(*this, PortKeys::kError, std::string());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus AppendTrajectory::failWithError(const std::string& msg)
{
  bt_utils::setOutput(*this, PortKeys::kError, msg);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rlc_executive
