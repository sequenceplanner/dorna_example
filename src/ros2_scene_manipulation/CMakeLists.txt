cmake_minimum_required(VERSION 3.5)
project(ros2_scene_manipulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_scene_manipulation_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  libs
)

# Service

add_executable(service_main
  src/main.cpp
  src/scene_manipulation_service.cpp
)

ament_target_dependencies(service_main
  ament_index_cpp
  rclcpp
  ros2_scene_manipulation_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS service_main
  DESTINATION lib/${PROJECT_NAME})


ament_package()
