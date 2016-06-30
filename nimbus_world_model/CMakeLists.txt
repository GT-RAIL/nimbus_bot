cmake_minimum_required(VERSION 2.8.3)
project(nimbus_world_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rail_manipulation_msgs
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  WorldModel.msg
  WorldObject.msg
)

generate_messages(DEPENDENCIES
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
add_executable(world_model src/world_model.cpp src/WorldObjectParticle.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(world_model
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(world_model rail_manipulation_msgs_gencpp nimbus_world_model_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS world_model
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
