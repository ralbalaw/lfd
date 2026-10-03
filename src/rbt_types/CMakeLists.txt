cmake_minimum_required(VERSION 3.16)
project(rbt_types VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(rbt_types_lib INTERFACE)
add_library(rbt_types::rbt_types ALIAS rbt_types_lib)
set_target_properties(rbt_types_lib PROPERTIES EXPORT_NAME rbt_types)

target_compile_features(rbt_types_lib INTERFACE cxx_std_20)

target_include_directories(rbt_types_lib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rbt_types_lib INTERFACE Eigen3::Eigen)

add_library(rbt_types_compile_flags OBJECT
  src/compile_flags.cpp
)
target_compile_features(rbt_types_compile_flags PRIVATE cxx_std_20)
target_include_directories(rbt_types_compile_flags PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(rbt_types_compile_flags PRIVATE Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS rbt_types_lib
  EXPORT export_${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(joint_trajectory_test
    test/joint_trajectory_test.cpp
  )
  target_link_libraries(joint_trajectory_test rbt_types::rbt_types)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rbt_types::)
ament_export_dependencies(Eigen3 eigen3_cmake_module)

ament_package()
