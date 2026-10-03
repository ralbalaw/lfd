cmake_minimum_required(VERSION 3.8)
project(rlc_scene_bridge LANGUAGES C CXX)

# Package level default (recommended in ROS workspaces)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tesseract_msgs REQUIRED)

# tesseract core packages are `build_type=cmake` (exported via CMake config)
find_package(tesseract_environment REQUIRED CONFIG)
find_package(tesseract_geometry REQUIRED CONFIG)
find_package(tesseract_scene_graph REQUIRED CONFIG)
find_package(tesseract_monitoring REQUIRED CONFIG)
find_package(rlc_utils REQUIRED)

add_library(rlc_scene_bridge_lib SHARED
  src/moveit_tesseract_bridge.cpp
  src/tesseract_env_sync.cpp
  src/tesseract_scene_utils.cpp
)

# Build-tree convenience and installed/exported name
add_library(rlc_scene_bridge::scene_bridge ALIAS rlc_scene_bridge_lib)
set_target_properties(rlc_scene_bridge_lib PROPERTIES EXPORT_NAME scene_bridge)

target_compile_features(rlc_scene_bridge_lib PUBLIC cxx_std_20)

target_include_directories(rlc_scene_bridge_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_scene_bridge_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_scene_bridge_lib
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
)

target_link_libraries(rlc_scene_bridge_lib
  Eigen3::Eigen
  tesseract::tesseract_environment
  tesseract::tesseract_geometry
  tesseract::tesseract_scene_graph
  tesseract_monitoring::tesseract_monitoring_environment_interface
)

add_executable(rlc_scene_bridge_node
  src/rlc_scene_bridge_node.cpp
)

target_compile_features(rlc_scene_bridge_node PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_scene_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_scene_bridge_node
  rclcpp
  tesseract_monitoring
  tesseract_msgs
)

target_link_libraries(rlc_scene_bridge_node
  rlc_scene_bridge_lib
  rlc_utils::tesseract
)

install(TARGETS rlc_scene_bridge_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS rlc_scene_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rlc_scene_bridge::)
ament_export_dependencies(
  Eigen3
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
  tesseract_environment
  tesseract_geometry
  tesseract_scene_graph
  tesseract_monitoring
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
