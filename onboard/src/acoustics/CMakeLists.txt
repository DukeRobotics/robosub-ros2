cmake_minimum_required(VERSION 3.16)
project(py_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AcousticSample.msg"
  "srv/GetAcousticData.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()