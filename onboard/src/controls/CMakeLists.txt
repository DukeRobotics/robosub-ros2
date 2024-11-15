cmake_minimum_required(VERSION 3.8)
project(controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(controls onboard/src/controls/controls/controls.cpp)
add_executable(pid_manager onboard/src/controls/controls/pid_manager.cpp)
add_executable(drc_pid onboard/src/controls/controls/drc_pid.cpp)
add_executable(controls_utils onboard/src/controls/controls/controls_utils.cpp)
add_executable(thruster_allocator onboard/src/controls/controls/thruster_allocator.cpp)

ament_target_dependencies(controls
  rclcpp
  custom_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  yaml_cpp_vendor
)

ament_target_dependencies(
  pid_manager
  rclcpp
  custom_msgs
)

ament_target_dependencies(
  drc_pid
  rclcpp
)

ament_target_dependencies(
  controls_utils
  rclcpp
)

ament_target_dependencies(
  thruster_allocator
  rclcpp
  custom_msgs
)

ament_package()
