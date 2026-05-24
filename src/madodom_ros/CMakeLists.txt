cmake_minimum_required(VERSION 3.14)
project(madodom_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(tf2_eigen         REQUIRED)
find_package(Eigen3            REQUIRED)
find_package(madodom_core      REQUIRED)
find_package(rosbag2_cpp       REQUIRED)
find_package(rosbag2_storage   REQUIRED)

# point_cloud_msg_wrapper may not be present; only use for field discovery,
# we parse fields manually in the node — just need the header for completeness.
find_package(point_cloud_msg_wrapper QUIET)

add_executable(madodom_node src/madodom_node.cpp)

target_include_directories(madodom_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(madodom_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
)

target_link_libraries(madodom_node
  madodom_core::madodom_core
)

target_compile_options(madodom_node PRIVATE -O3 -march=native -Wall -Wextra)

# ── Offline bag runner ────────────────────────────────────────────────────────
add_executable(madodom_offline src/madodom_offline.cpp)

target_include_directories(madodom_offline PRIVATE ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(madodom_offline
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
)

target_link_libraries(madodom_offline madodom_core::madodom_core)
target_compile_options(madodom_offline PRIVATE -O3 -march=native -Wall -Wextra)

install(TARGETS madodom_node madodom_offline DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/publish_gt.py DESTINATION lib/${PROJECT_NAME})

ament_package()
