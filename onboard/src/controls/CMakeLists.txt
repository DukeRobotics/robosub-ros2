cmake_minimum_required(VERSION 3.14.4)
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
find_package(yaml-cpp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(fmt REQUIRED)

include_directories(/usr/local/include/osqp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/controls ${custom_msgs_INCLUDE_DIRS})

set(SOURCES
  src/controls.cpp
  src/thruster_allocator.cpp
  src/pid_manager.cpp
  src/drc_pid.cpp
  src/controls_utils.cpp
)
add_executable(controls ${SOURCES})

set(CONTROLS_INCLUDE_DIR ~/dev/robosub-ros2/onboard/src/controls/include/controls)
target_include_directories(controls PUBLIC ${CONTROLS_INCLUDE_DIR})

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
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(controls fmt::fmt yaml-cpp Eigen3::Eigen OsqpEigen::OsqpEigen)

install(TARGETS controls DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()


