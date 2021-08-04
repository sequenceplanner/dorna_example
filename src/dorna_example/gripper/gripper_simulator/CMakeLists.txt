cmake_minimum_required(VERSION 3.5)
project(gripper_simulator)

find_package(ament_cmake REQUIRED)

include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(sp_messages
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
