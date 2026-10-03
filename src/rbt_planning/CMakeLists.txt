cmake_minimum_required(VERSION 3.16)
project(rbt_planning VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Default policy for this package (helps consistency for any future targets)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies (exported as part of the public interface via target_link_libraries PUBLIC)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED CONFIG)
find_package(tesseract_environment REQUIRED)
find_package(trajopt REQUIRED)
find_package(rbt_types REQUIRED)

add_library(rbt_planning
  src/ompl_solver.cpp
  src/trajopt_planner.cpp
  src/trajopt_solver.cpp
  src/utils.cpp
)
add_library(rbt_planning::rbt_planning ALIAS rbt_planning)

# Public contract: consumers compile with C++20 when they use your headers
target_compile_features(rbt_planning PUBLIC cxx_std_20)

target_include_directories(rbt_planning
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(rbt_planning
  PUBLIC
    ompl::ompl
    trajopt::trajopt
    tesseract::tesseract_environment
    Eigen3::Eigen
    rbt_types::rbt_types
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rbt_planning PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_target_properties(rbt_planning PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  EXPORT_NAME rbt_planning
)

# --------------------------
# Install: library + headers
# --------------------------

install(TARGETS rbt_planning
  EXPORT rbt_planningTargets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# --------------------------
# Package config (find_package)
# --------------------------

# Export targets file
install(EXPORT rbt_planningTargets
  NAMESPACE rbt_planning::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbt_planning
)

# Generate and install the config + version files
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rbt_planningConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_planningConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbt_planning
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_planningConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_planningConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_planningConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbt_planning
)
