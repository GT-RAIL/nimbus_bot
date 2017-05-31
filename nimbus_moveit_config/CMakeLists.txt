cmake_minimum_required(VERSION 2.8.3)
project(nimbus_moveit_config)

find_package(catkin REQUIRED
  actionlib
  cmake_modules
  control_msgs
  ecl_geometry
  message_generation
  geometry_msgs
  roscpp
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  rail_manipulation_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  )

find_package(Eigen REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(nimbus_moveit src/nimbus_moveit.cpp include/nimbus_moveit_config/nimbus_moveit.h)
add_executable(common_actions src/common_actions.cpp)
add_executable(primitive_actions src/primitive_actions.cpp)
add_executable(jaco_trajectory_controller src/jaco_trajectory_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(nimbus_moveit ${catkin_LIBRARIES})
target_link_libraries(common_actions ${catkin_LIBRARIES})
target_link_libraries(primitive_actions ${catkin_LIBRARIES})
target_link_libraries(jaco_trajectory_controller ${catkin_LIBRARIES})

add_dependencies(nimbus_moveit rail_manipulation_msgs_gencpp)
add_dependencies(common_actions rail_manipulation_msgs_gencpp)
add_dependencies(primitive_actions rail_manipulation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS nimbus_moveit common_actions
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
