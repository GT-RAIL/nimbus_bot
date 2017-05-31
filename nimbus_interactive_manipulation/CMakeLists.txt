cmake_minimum_required(VERSION 2.8.3)
project(nimbus_interactive_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  interactive_markers
  message_generation
  move_base_msgs
  rail_segmentation
  rail_pick_and_place_msgs
  roscpp
  sensor_msgs
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  CreateNavidgetSphere.srv
)

add_action_files(
  FILES
  SpecifiedGrasp.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(nimbus_interactive_manipulation src/nimbus_interactive_manipulation.cpp)
add_executable(nimbus_object_viewer src/nimbusObjectViewer.cpp)
add_executable(nimbus_6dof_planning src/nimbus6dofPlanning.cpp)
add_executable(nimbus_6dof_vis src/nimbus6dofVis.cpp)
add_executable(nimbus_navidget src/nimbusNavidget.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(nimbus_interactive_manipulation
  ${catkin_LIBRARIES}
)
target_link_libraries(nimbus_object_viewer
  ${catkin_LIBRARIES}
  )
target_link_libraries(nimbus_6dof_planning
  ${catkin_LIBRARIES}
  )
target_link_libraries(nimbus_6dof_vis
  ${catkin_LIBRARIES}
  )
target_link_libraries(nimbus_navidget
  ${catkin_LIBRARIES}
  )

## Add cmake target dependencies of the executable/library
add_dependencies(nimbus_interactive_manipulation rail_pick_and_place_msgs_gencpp wpi_jaco_msgs_gencpp)
add_dependencies(nimbus_object_viewer rail_pick_and_place_msgs_gencpp)
add_dependencies(nimbus_6dof_planning rail_manipulation_msgs_gencpp wpi_jaco_msgs_gencpp)
add_dependencies(nimbus_navidget rail_manipulation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS nimbus_interactive_manipulation nimbus_6dof_planning
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
