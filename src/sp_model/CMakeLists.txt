cmake_minimum_required(VERSION 3.5)
project(sp_model)

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
         )

# install your binaries
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/cylinders
  ${CMAKE_SOURCE_DIR}/target/release/cylinders2
  ${CMAKE_SOURCE_DIR}/target/release/cylinders2_structs
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros
find_package(ament_cmake REQUIRED)
ament_package()
