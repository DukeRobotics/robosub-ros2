cmake_minimum_required(VERSION 3.8)
project(dvl_pathfinder LANGUAGES C CXX)  # Enable both C and C++

set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(CXX_SOURCES
  src/dvl_pathfinder.cpp
)

set(C_SOURCES
  src/PDD_Decoder.c
  src/PDD_Ensemble.c
  src/PDD_Helper.c
)

add_executable(dvl_pathfinder ${CXX_SOURCES} ${C_SOURCES})

target_include_directories(dvl_pathfinder PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dvl_pathfinder
)

target_link_libraries(dvl_pathfinder
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

# Explicitly set the language for C files
set_source_files_properties(${C_SOURCES} PROPERTIES LANGUAGE C)

install(TARGETS dvl_pathfinder DESTINATION lib/${PROJECT_NAME})

ament_package()