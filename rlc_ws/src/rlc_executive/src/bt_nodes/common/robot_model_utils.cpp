#include "rlc_executive/bt_nodes/common/robot_model_utils.hpp"

#include <stdexcept>

namespace rlc_executive::bt_utils
{
const std::vector<std::string>&
getActiveJointNames(const moveit::core::RobotModel& robot_model,
                    const std::string& group_name)
{
  const moveit::core::JointModelGroup* joint_group =
      robot_model.getJointModelGroup(group_name);

  if (joint_group == nullptr)
  {
    throw std::invalid_argument("unknown MoveIt joint group: " + group_name);
  }

  if (!joint_group->isSingleDOFJoints())
  {
    throw std::invalid_argument("joint group must contain only single-DOF joints: " +
                                group_name);
  }

  return joint_group->getActiveJointModelNames();
}
}  // namespace rlc_executive::bt_utils