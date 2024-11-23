cmake_minimum_required(VERSION 3.8)
project(controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(/usr/local/include/osqp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${custom_msgs_INCLUDE_DIRS})

add_executable(controls src/controls.cpp)
add_executable(pid_manager src/pid_manager.cpp)
add_executable(drc_pid src/drc_pid.cpp)
add_executable(controls_utils src/controls_utils.cpp)
add_executable(thruster_allocator src/thruster_allocator.cpp)

set(CONTROLS_INCLUDE_DIR ~/dev/robosub-ros2/onboard/src/controls/include)
target_include_directories(controls PUBLIC ${CONTROLS_INCLUDE_DIR})
target_include_directories(controls_utils PUBLIC ${CONTROLS_INCLUDE_DIR})
target_include_directories(drc_pid PUBLIC ${CONTROLS_INCLUDE_DIR})
target_include_directories(pid_manager PUBLIC ${CONTROLS_INCLUDE_DIR})
target_include_directories(thruster_allocator PUBLIC ${CONTROLS_INCLUDE_DIR})

ament_target_dependencies(
  controls
  rclcpp
  std_msgs
  custom_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  yaml_cpp_vendor
  ament_index_cpp
)

ament_target_dependencies(
  pid_manager
  rclcpp
  tf2
  Eigen3
  custom_msgs
  ament_index_cpp
)

ament_target_dependencies(
  drc_pid
  rclcpp
  tf2
  Eigen3
  custom_msgs
  ament_index_cpp
)

ament_target_dependencies(
  controls_utils
  custom_msgs
  geometry_msgs
  rclcpp
  ament_index_cpp
  tf2
  tf2_geometry_msgs
  Eigen3
  yaml_cpp_vendor
)

ament_target_dependencies(
  thruster_allocator
  rclcpp
  custom_msgs
  Eigen3
  OsqpEigen
)

ament_package()
