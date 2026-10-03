#include <exception>

#include <rclcpp/rclcpp.hpp>

#include <rlc_planner/time_parameterization/moveit_time_parameterizer.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = rlc_planner::MoveItTimeParameterizer::create();
    rclcpp::spin(node);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_FATAL(rclcpp::get_logger("time_parameterization_server"),
                 "Failed to start time_parameterization_server: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
