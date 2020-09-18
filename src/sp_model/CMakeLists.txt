cmake_minimum_required(VERSION 3.5)
project(sp_model)

# create our own AMENT_PREFIX_PATH just for this package. this allows
# us to build our stuff without colcon by pulling in all ros deps
function(ros_deps)
  foreach(f ${ARGN})
    find_package(${f} REQUIRED)
    list(APPEND AMENT_HACK_PATH "${${f}_DIR}/../../../")
  endforeach()
  # potential platform issue below
  string (REPLACE ";" ":" AMENT_HACK_PATH_STR "${AMENT_HACK_PATH}")
  set(ENV{AMENT_HACK_PATH} ${AMENT_HACK_PATH_STR})
endfunction()

# put ros package dependencies here.
ros_deps(std_msgs
         rcl
         sp_operator_msgs
         sp_messages
         control_box_msgs
         camera_msgs
         robot_msgs
         gripper_msgs
        )

# this works fine except that it will be called both on make all and again on make install...
option(CARGO_CLEAN "Invoke cargo clean before building" OFF)
if(CARGO_CLEAN)
      add_custom_target(dummy_target ALL
            COMMAND "cmake" "-E" "env" "cargo" "clean"
            COMMAND "cmake" "-E" "env" "AMENT_PREFIX_PATH=$ENV{AMENT_HACK_PATH}" "cargo" "build" "--release"
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            )
else()
        add_custom_target(dummy_target ALL
            COMMAND "cmake" "-E" "env" "AMENT_PREFIX_PATH=$ENV{AMENT_HACK_PATH}" "cargo" "build" "--release"
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            )
endif(CARGO_CLEAN)
unset(CARGO_CLEAN CACHE)

install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/cylinders
  ${CMAKE_SOURCE_DIR}/target/release/cylinders2
  ${CMAKE_SOURCE_DIR}/target/release/cylinders2_structs
  DESTINATION lib/${PROJECT_NAME}
)

# we need this
find_package(ament_cmake REQUIRED)
ament_package()
