cmake_minimum_required(VERSION 3.16)
project(rbt_irl VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rbt_types REQUIRED)

add_library(rbt_irl_lib INTERFACE)
add_library(rbt_irl::rbt_irl ALIAS rbt_irl_lib)
set_target_properties(rbt_irl_lib PROPERTIES EXPORT_NAME rbt_irl)

target_compile_features(rbt_irl_lib INTERFACE cxx_std_20)

target_include_directories(rbt_irl_lib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rbt_irl_lib INTERFACE
  rbt_types::rbt_types
)

install(TARGETS rbt_irl_lib
  EXPORT export_${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rbt_irl::)
ament_export_dependencies(rbt_types)

ament_package()
