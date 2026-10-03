cmake_minimum_required(VERSION 3.22)
project(rlc_frontend LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rlc_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(
  collect_demonstration_parameters
  config/collect_demonstration_parameters.yaml
)

add_library(rlc_frontend_lib SHARED
  src/frontend/frontend_config_parameters.cpp
  src/frontend/joy_mapping.cpp
  src/frontend/moveit_servo_client.cpp
  src/frontend/servo_joy_frontend.cpp
  src/frontend/teleoperation_controller.cpp
  src/frontend/teleoperation_session.cpp
)
add_library(rlc_frontend::frontend ALIAS rlc_frontend_lib)

set_target_properties(rlc_frontend_lib PROPERTIES EXPORT_NAME frontend)

target_compile_features(rlc_frontend_lib PUBLIC cxx_std_20)

target_include_directories(rlc_frontend_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_frontend_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_frontend_lib
  PUBLIC
    control_msgs
    geometry_msgs
    moveit_msgs
    rclcpp
    sensor_msgs
    std_srvs
)

target_link_libraries(rlc_frontend_lib
  PUBLIC
    collect_demonstration_parameters
)

add_library(rlc_frontend_server SHARED
  src/server/collect_demonstration_server.cpp
  src/server/recording/joint_demonstration_recorder.cpp
)
add_library(rlc_frontend::server ALIAS rlc_frontend_server)

set_target_properties(rlc_frontend_server PROPERTIES EXPORT_NAME server)

target_compile_features(rlc_frontend_server PUBLIC cxx_std_20)

target_include_directories(rlc_frontend_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/server
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_frontend_server PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_frontend_server
  PUBLIC
    moveit_msgs
    rclcpp
    rclcpp_action
    rlc_interfaces
)

target_link_libraries(rlc_frontend_server
  PUBLIC
    rlc_frontend::frontend
  PRIVATE
    ${sensor_msgs_TARGETS}
)

add_executable(collect_demonstration_server
  src/server/collect_demonstration_main.cpp
)

target_compile_features(collect_demonstration_server PRIVATE cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(collect_demonstration_server PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(collect_demonstration_server rclcpp)

target_link_libraries(collect_demonstration_server rlc_frontend::server)

install(TARGETS
  rlc_frontend_lib
  rlc_frontend_server
  collect_demonstration_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS collect_demonstration_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rlc_frontend::)
ament_export_dependencies(
  control_msgs
  generate_parameter_library
  geometry_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  rlc_interfaces
  sensor_msgs
  std_srvs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(joint_demonstration_recorder_test
    test/server/recording/joint_demonstration_recorder_test.cpp
  )
  target_include_directories(joint_demonstration_recorder_test
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/server
  )
  ament_target_dependencies(joint_demonstration_recorder_test
    moveit_msgs
    rclcpp
    sensor_msgs
  )
  target_link_libraries(joint_demonstration_recorder_test
    rlc_frontend::server
  )
endif()

ament_package()
