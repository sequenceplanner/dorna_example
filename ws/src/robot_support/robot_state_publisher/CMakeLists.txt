cmake_minimum_required(VERSION 3.5)
project(robot_state_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME}_node SHARED
  src/robot_state_publisher.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  builtin_interfaces
  geometry_msgs
  kdl_parser
  orocos_kdl
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_ros
  urdf
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "robot_state_publisher::RobotStatePublisher"
  EXECUTABLE robot_state_publisher)

install(
  TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(builtin_interfaces orocos_kdl rclcpp sensor_msgs std_msgs tf2_ros urdf)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)
ament_package()
