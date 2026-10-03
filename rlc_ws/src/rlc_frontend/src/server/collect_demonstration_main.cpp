#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rlc_frontend/server/collect_demonstration_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;

  rclcpp::spin(std::make_shared<rlc_frontend::CollectDemonstrationServer>(node_options));
  rclcpp::shutdown();
  return 0;
}
