#include "rlc_executive/executive/bt_node_registration.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include "rlc_executive/bt_nodes/demonstration/append_trajectory.hpp"
#include "rlc_executive/bt_nodes/demonstration/build_joint_endpoints.hpp"
#include "rlc_executive/bt_nodes/demonstration/record_servo_demo.hpp"
#include "rlc_executive/bt_nodes/irl/build_features.hpp"
#include "rlc_executive/bt_nodes/irl/fit_max_ent_irl.hpp"
#include "rlc_executive/bt_nodes/motion/build_motion_plan_request.hpp"
#include "rlc_executive/bt_nodes/motion/execute_trajectory.hpp"
#include "rlc_executive/bt_nodes/motion/load_pose.hpp"
#include "rlc_executive/bt_nodes/motion/moveit_plan.hpp"
#include "rlc_executive/bt_nodes/motion/tesseract_plan.hpp"
#include "rlc_executive/bt_nodes/motion/time_parameterize_moveit.hpp"

namespace rlc_executive
{
void registerExecutiveBtNodes(BT::BehaviorTreeFactory& factory,
                              const rclcpp::Node::SharedPtr& node,
                              const rclcpp::Logger& logger, const ExecutiveConfig& config,
                              const PlanningConfigRegistry& planning_configs,
                              RobotResources& robot_resources)
{
  const PlanningConfig* default_planning_config = &planning_configs.defaultConfig();
  const std::string& monitor_namespace = config.monitor.monitor_namespace;

  tesseract_monitoring::ROSEnvironmentMonitorInterface* env_monitor_interface =
      robot_resources.environmentMonitor();

  factory.registerNodeType<LoadPose>("LoadPose", logger);
  factory.registerNodeType<BuildMotionPlanRequest>("BuildMotionPlanRequest", logger);

  factory.registerNodeType<TesseractPlan>("TesseractPlan", logger, env_monitor_interface,
                                          default_planning_config, monitor_namespace);

  factory.registerNodeType<AppendTrajectory>("AppendTrajectory");
  factory.registerNodeType<BuildJointEndpoints>("BuildJointEndpoints");

  factory.registerNodeType<FitMaxEntIRL>("FitMaxEntIRL", logger, env_monitor_interface,
                                         default_planning_config, monitor_namespace);

  factory.registerNodeType<BuildFeatures>("BuildFeatures", logger, env_monitor_interface,
                                          monitor_namespace);

  BT::RosNodeParams demo_params(node);
  demo_params.default_port_value = config.demonstration_collection.action_name;
  factory.registerNodeType<RecordServoDemo>("RecordServoDemo", demo_params);

  BT::RosNodeParams move_group_params(node);
  move_group_params.default_port_value = config.move_group.action_name;
  factory.registerNodeType<MoveItPlan>("MoveItPlan", move_group_params, &planning_configs);

  BT::RosNodeParams exec_params(node);
  exec_params.default_port_value = config.execution.execute_traj_action_name;
  factory.registerNodeType<ExecuteTrajectory>("ExecuteTrajectory", exec_params,
                                              robot_resources.model());

  factory.registerNodeType<TimeParameterizeMoveIt>("TimeParameterizeMoveIt", logger,
                                                   robot_resources.model(),
                                                   robot_resources.referenceState(),
                                                   &planning_configs);
}
}  // namespace rlc_executive
