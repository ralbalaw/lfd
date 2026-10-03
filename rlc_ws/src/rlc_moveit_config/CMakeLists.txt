cmake_minimum_required(VERSION 3.5)

# NOTE: Modified from upstream franka_ros2 by hri_rlc.

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(rlc_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES LICENSE NOTICE README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_pytest_test(test_fr3_tesseract_srdf
    test/test_fr3_tesseract_srdf.py
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
