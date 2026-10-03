cmake_minimum_required(VERSION 3.22)
project(rlc_executive LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rlc_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(MPI REQUIRED COMPONENTS C)
find_package(rbt_irl REQUIRED)
find_package(rbt_features REQUIRED)
find_package(rbt_types REQUIRED)
find_package(rbt_planning REQUIRED CONFIG)
find_package(rlc_utils REQUIRED)

generate_parameter_library(
  bt_executive_parameters
  config/bt_executive_parameters.yaml
)

# Core library (no BT leaf nodes here)
add_library(rlc_executive_core SHARED
  src/config/executive_config_loader.cpp
  src/config/planning_config_loader.cpp
  src/config/planning_config_registry.cpp
  src/executive/robot_resources.cpp
)

target_compile_features(rlc_executive_core PUBLIC cxx_std_20)

target_include_directories(rlc_executive_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_executive_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_executive_core
  PUBLIC
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    rclcpp
    tf2_ros
    yaml_cpp_vendor
    rlc_utils
)

target_link_libraries(rlc_executive_core
  PUBLIC
    bt_executive_parameters
  PRIVATE
    yaml-cpp::yaml-cpp
)

# BT node library (all built-in BT leaf nodes live here)
add_library(rlc_executive_bt_nodes SHARED
  src/bt_nodes/common/node_utils.cpp
  src/bt_nodes/common/robot_model_utils.cpp
  src/bt_nodes/common/trajectory_conversion.cpp
  src/bt_nodes/demonstration/append_trajectory.cpp
  src/bt_nodes/demonstration/build_joint_endpoints.cpp
  src/bt_nodes/demonstration/record_servo_demo.cpp
  src/bt_nodes/irl/build_features.cpp
  src/bt_nodes/irl/fit_max_ent_irl.cpp
  src/bt_nodes/motion/build_motion_plan_request.cpp
  src/bt_nodes/motion/execute_trajectory.cpp
  src/bt_nodes/motion/load_pose.cpp
  src/bt_nodes/motion/moveit_plan.cpp
  src/bt_nodes/motion/tesseract_plan.cpp
  src/bt_nodes/motion/time_parameterize_moveit.cpp
)

target_compile_features(rlc_executive_bt_nodes PUBLIC cxx_std_20)

target_include_directories(rlc_executive_bt_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_executive_bt_nodes PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_executive_bt_nodes
  PUBLIC
    behaviortree_cpp
    behaviortree_ros2
    rclcpp
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    rlc_interfaces
    rclcpp_action
    sensor_msgs
    rbt_irl
    rbt_types
    rlc_utils
)

# Most BT nodes use executive-owned runtime services.
target_link_libraries(rlc_executive_bt_nodes
  PUBLIC
    rlc_executive_core
    rbt_planning::rbt_planning
  PRIVATE
    rbt_features::rbt_features
)

# Runner library depends on the BT node implementation library for direct
# registration of built-in nodes.
add_library(${PROJECT_NAME} SHARED
  src/executive/bt_node_registration.cpp
  src/executive/bt_executive.cpp
  src/executive/tree_runner.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    ament_index_cpp
    behaviortree_cpp
    behaviortree_ros2
    rclcpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rlc_executive_core
  PRIVATE
    rlc_executive_bt_nodes
)

# Executable
add_executable(rlc_bt_executive
  src/executive/main.cpp
)

target_compile_features(rlc_bt_executive PRIVATE cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_bt_executive PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_bt_executive rclcpp)
target_link_libraries(rlc_bt_executive ${PROJECT_NAME})

# Install
install(TARGETS
  ${PROJECT_NAME}
  rlc_executive_core
  bt_executive_parameters
  rlc_executive_bt_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS rlc_bt_executive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY bt_trees/
  DESTINATION share/${PROJECT_NAME}/bt_trees
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  behaviortree_cpp
  behaviortree_ros2
  builtin_interfaces
  generate_parameter_library
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  rlc_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_ros
  yaml_cpp_vendor
  yaml-cpp
  rbt_irl
  rbt_types
  rlc_utils
  rbt_planning
)

ament_package()
