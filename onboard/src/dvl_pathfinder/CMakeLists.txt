cmake_minimum_required(VERSION 3.8)
project(dvl_pathfinder LANGUAGES C CXX)  # Enable both C and C++

set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${custom_msgs_INCLUDE_DIRS}
)

set(CXX_SOURCES
  src/dvl_pathfinder.cpp
)

set(C_SOURCES
  src/PDD/PDD_Decoder.c
  src/PDD/PDD_Ensemble.c
  src/PDD/PDD_Helper.c
)

add_executable(dvl_pathfinder ${CXX_SOURCES} ${C_SOURCES})

target_include_directories(dvl_pathfinder PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dvl_pathfinder
)

target_link_libraries(dvl_pathfinder
  fmt::fmt
  rclcpp::rclcpp
  yaml-cpp::yaml-cpp
  ${ament_index_cpp_TARGETS}
  ${custom_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Explicitly set the language for C files
set_source_files_properties(${C_SOURCES} PROPERTIES LANGUAGE C)

install(TARGETS dvl_pathfinder DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()