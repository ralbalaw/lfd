cmake_minimum_required(VERSION 3.8)
project(rlc_planner LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rlc_interfaces REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(rlc_utils REQUIRED)

# Non-ament (CMake) dependencies
find_package(rbt_planning REQUIRED CONFIG)
find_package(trajopt REQUIRED CONFIG)
find_package(tesseract_environment REQUIRED CONFIG)

add_library(rlc_planner_plugin SHARED
  src/trajopt/trajopt_interface.cpp
  src/trajopt/trajopt_planner_manager.cpp
  src/trajopt/trajopt_planning_context.cpp
)

target_compile_features(rlc_planner_plugin PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_planner_plugin PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(rlc_planner_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(rlc_planner_plugin
  rclcpp
  pluginlib
  moveit_core
  moveit_msgs
  tesseract_monitoring
)

target_link_libraries(rlc_planner_plugin
  Eigen3::Eigen
  rbt_planning::rbt_planning
  trajopt::trajopt
  tesseract::tesseract_environment
  rlc_utils::tesseract
)

add_library(rlc_planner_time_parameterization SHARED
  src/time_parameterization/moveit_time_parameterizer.cpp
)

target_compile_features(rlc_planner_time_parameterization PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_planner_time_parameterization PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(rlc_planner_time_parameterization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(rlc_planner_time_parameterization
  rclcpp
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rlc_interfaces
  sensor_msgs
)

add_executable(time_parameterization_server
  src/time_parameterization/main.cpp
)

target_compile_features(time_parameterization_server PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(time_parameterization_server PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(time_parameterization_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(time_parameterization_server
  rclcpp
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rlc_interfaces
  sensor_msgs
)

target_link_libraries(time_parameterization_server
  rlc_planner_time_parameterization
)

pluginlib_export_plugin_description_file(moveit_core rlc_planner_plugins.xml)

install(TARGETS
  rlc_planner_plugin
  rlc_planner_time_parameterization
  DESTINATION lib
)

install(TARGETS time_parameterization_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES rlc_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
