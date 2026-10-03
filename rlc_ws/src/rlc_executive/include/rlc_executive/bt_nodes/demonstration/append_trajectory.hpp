#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include "rbt_types/joint_trajectory.hpp"
#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{

/**
 * @brief Appends one recorded trajectory to a trajectory-set blackboard value.
 *
 * @details
 * This node reads one input trajectory and appends it to the existing
 * `demonstrations` set. If the set is empty, a new set is created.
 *
 * Input ports:
 * - `trajectory`: Trajectory to append.
 *
 * In/out ports:
 * - `demonstrations`: Existing trajectory set to extend; may be empty.
 *
 * Output ports:
 * - `error`: Human-readable failure text.
 */
class AppendTrajectory final : public BT::SyncActionNode
{
public:
  using JointTrajectory = rbt_types::JointTrajectory;
  using JointTrajectories = rbt_types::JointTrajectories;

  /**
   * @brief Constructs the append-trajectory BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   */
  AppendTrajectory(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing the single trajectory input, the
   * accumulated demonstration set, and an error output.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<const JointTrajectory>>(PortKeys::kTrajectory),
      BT::BidirectionalPort<std::shared_ptr<const JointTrajectories>>(
          PortKeys::kDemonstrations, std::shared_ptr<const JointTrajectories>(), ""),
      BT::OutputPort<std::string>(PortKeys::kError),
    };
  }

  /**
   * @brief Names of BT ports used by AppendTrajectory.
   */
  struct PortKeys
  {
    static inline const std::string kTrajectory = "trajectory";
    static inline const std::string kDemonstrations = "demonstrations";
    static inline const std::string kError = "error";
  };

  /**
   * @brief Appends the input trajectory to the current trajectory set.
   * @return `SUCCESS` after the set is updated, otherwise `FAILURE`.
   */
  BT::NodeStatus tick() override;

private:
  BT::NodeStatus failWithError(const std::string& msg);
};

}  // namespace rlc_executive
