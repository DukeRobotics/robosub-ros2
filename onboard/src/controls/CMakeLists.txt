cmake_minimum_required(VERSION 3.14.4)
project(controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${custom_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/osqp
)

set(SOURCES
  src/controls.cpp
  src/thruster_allocator.cpp
  src/pid_manager.cpp
  src/drc_pid.cpp
  src/controls_utils.cpp
)
add_executable(controls ${SOURCES})

target_include_directories(controls PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/controls)

target_link_libraries(controls
  Eigen3::Eigen
  fmt::fmt
  OsqpEigen::OsqpEigen
  rclcpp::rclcpp
  yaml-cpp
  ${ament_index_cpp_TARGETS}
  ${custom_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${tf2_TARGETS}
)

target_precompile_headers(controls
  PRIVATE
    <fmt/core.h>
    <rclcpp/rclcpp.hpp>
    <tf2/LinearMath/Matrix3x3.h>
    <tf2/LinearMath/Vector3.h>
    <tf2/LinearMath/Quaternion.h>
    <tf2_ros/transform_listener.h>
    <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
    <Eigen/Dense>
    <yaml-cpp/yaml.h>
    [["OsqpEigen/OsqpEigen.h"]]
)

install(TARGETS controls DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_package()


