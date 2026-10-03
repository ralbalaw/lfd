#pragma once
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>

namespace rlc_executive::bt_utils
{
const std::vector<std::string>&
getActiveJointNames(const moveit::core::RobotModel& robot_model,
                    const std::string& group_name);
}  // namespace rlc_executive::bt_utils