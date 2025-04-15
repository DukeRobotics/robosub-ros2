cmake_minimum_required(VERSION 3.8)
project(dvl_pathfinder LANGUAGES CXX)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(SOURCES
  src/dvl_pathfinder.cpp
  src/PDD_Decoder.c
  src/PDD_Ensemble.c
  src/PDD_Helper.c
)
add_executable(dvl_pathfinder ${SOURCES})

target_include_directories(dvl_pathfinder PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dvl_pathfinder
)

target_link_libraries(dvl_pathfinder
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

target_precompile_headers(dvl_pathfinder
  PRIVATE
    <rclcpp/rclcpp.hpp>
)

install(TARGETS dvl_pathfinder DESTINATION lib/${PROJECT_NAME})

ament_package()
