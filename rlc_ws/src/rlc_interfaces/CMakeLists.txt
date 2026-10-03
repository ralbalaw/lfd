cmake_minimum_required(VERSION 3.8)
project(rlc_interfaces LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CollectDemonstration.action"
  "srv/TimeParameterizeTrajectory.srv"
  DEPENDENCIES
    moveit_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
