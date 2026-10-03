cmake_minimum_required(VERSION 3.16)
project(rbt_features VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Tesseract's environment package pulls PCL/VTK, and the VTK import expects
# MPI::MPI_C to exist during configure. On Debian/Ubuntu OpenMPI installs,
# seeding the MPI header directory lets FindMPI create that imported target.
find_path(RBT_FEATURES_MPI_C_HEADER_DIR
  NAMES mpi.h
  HINTS
    /usr/lib/x86_64-linux-gnu/openmpi/include
    /usr/lib64/openmpi/include
    /usr/lib/openmpi/include
)

if(RBT_FEATURES_MPI_C_HEADER_DIR
   AND (NOT DEFINED MPI_C_HEADER_DIR OR MPI_C_HEADER_DIR STREQUAL "MPI_C_HEADER_DIR-NOTFOUND"))
  set(MPI_C_HEADER_DIR "${RBT_FEATURES_MPI_C_HEADER_DIR}" CACHE PATH
    "MPI C header directory")
endif()

find_package(MPI REQUIRED COMPONENTS C)
find_package(rbt_types REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS core)
find_package(tesseract_environment REQUIRED)

add_library(rbt_features_lib
  src/features/distance_feature.cpp
  src/features/orientation_feature.cpp
  src/queries/tesseract_distance_query.cpp
  src/queries/tesseract_orientation_query.cpp
)
add_library(rbt_features::rbt_features ALIAS rbt_features_lib)

set_target_properties(rbt_features_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  EXPORT_NAME rbt_features
)

target_compile_features(rbt_features_lib PUBLIC cxx_std_20)

target_include_directories(rbt_features_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(rbt_features_lib
  PUBLIC
    Eigen3::Eigen
    rbt_types::rbt_types
    tesseract::tesseract_collision_core
    tesseract::tesseract_environment
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rbt_features_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS rbt_features_lib
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rbt_features::)
ament_export_dependencies(Eigen3 eigen3_cmake_module rbt_types tesseract_collision
  tesseract_environment)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
