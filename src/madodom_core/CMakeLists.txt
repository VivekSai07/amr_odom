cmake_minimum_required(VERSION 3.14)
project(madodom_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Core library (Eigen only, no ROS) ─────────────────────────────────────────
add_library(madodom_core SHARED
  src/mad_tree.cpp
  src/icp.cpp
  src/local_map.cpp
  src/odometry.cpp
)

target_include_directories(madodom_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_options(madodom_core PRIVATE -O3 -march=native -Wall -Wextra)

# ── Unit tests ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mad_tree test/test_mad_tree.cpp)
  target_link_libraries(test_mad_tree madodom_core)

  ament_add_gtest(test_icp test/test_icp.cpp)
  target_link_libraries(test_icp madodom_core)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS madodom_core EXPORT madodom_coreTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT madodom_coreTargets
  FILE madodom_coreTargets.cmake
  NAMESPACE madodom_core::
  DESTINATION lib/cmake/madodom_core
)

ament_export_targets(madodom_coreTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
