cmake_minimum_required(VERSION 3.5)
project(sp_model)

find_package(ament_cmake REQUIRED)

include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(std_msgs
          sp_operator_msgs
          sp_messages
          control_box_msgs
          camera_msgs
          robot_msgs
          gripper_msgs
          rcl
          rcl_action             # as of r2r 0.1.0, we also need the action api
          rmw_fastrtps_cpp       # (needed to build with RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
          FastRTPS               # (needed to build with RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
         )

# install your binaries
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros
ament_package()
