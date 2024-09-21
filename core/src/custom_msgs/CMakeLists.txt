cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB msg_sources RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
file(GLOB srv_sources RELATIVE ${PROJECT_SOURCE_DIR} srv/*.srv)

message("Globbing messages ${msg_sources}")
message("Globbing services ${srv_sources}")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_sources}
  ${srv_sources}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
