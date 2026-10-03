#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include "rbt_types/joint_endpoints.hpp"
#include "rbt_types/joint_trajectory.hpp"

namespace rlc_executive
{

/**
 * @brief Builds return-planning endpoints from a recorded joint trajectory.
 *
 * @details
 * The output endpoint pair is ordered for returning from the recorded trajectory
 * end to its beginning:
 * - `start`: last trajectory waypoint [rad], size = trajectory dimension.
 * - `goal`: first trajectory waypoint [rad], size = trajectory dimension.
 *
 * Input ports:
 * - `trajectory`: Source joint trajectory.
 *
 * Output ports:
 * - `joint_endpoints`: Start and goal joint positions for planning.
 */
class BuildJointEndpoints final : public BT::SyncActionNode
{
public:
  using JointEndpoints = rbt_types::JointEndpoints;
  using JointTrajectory = rbt_types::JointTrajectory;

  /**
   * @brief Constructs the joint-endpoint builder BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   */
  BuildJointEndpoints(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing trajectory input and endpoint output.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<const JointTrajectory>>(PortKeys::kTrajectory),
      BT::OutputPort<JointEndpoints>(PortKeys::kJointEndpoints),
    };
  }

  /**
   * @brief Names of BT ports used by BuildJointEndpoints.
   */
  struct PortKeys
  {
    static inline const std::string kTrajectory = "trajectory";
    static inline const std::string kJointEndpoints = "joint_endpoints";
  };

  /**
   * @brief Builds return endpoints and writes them to the output port.
   * @return `SUCCESS` after writing the endpoint pair.
   */
  BT::NodeStatus tick() override;
};

}  // namespace rlc_executive
