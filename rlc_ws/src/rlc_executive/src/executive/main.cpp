#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/executive/bt_executive.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor ros_executor(rclcpp::ExecutorOptions{}, 2);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("bt_executive", node_options);
  auto bt_executive = std::make_unique<rlc_executive::BtExecutive>(
      node, [](BT::NodeStatus) { rclcpp::shutdown(); });
  bt_executive->start();

  ros_executor.add_node(node);
  ros_executor.spin();

  bt_executive.reset();
  rclcpp::shutdown();
  return 0;
}
