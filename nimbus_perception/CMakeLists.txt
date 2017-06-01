cmake_minimum_required(VERSION 2.8.3)
project(nimbus_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  pcl_conversions
  pcl_ros
  rail_manipulation_msgs
  roscpp
  shape_msgs
  rospy
  std_msgs
  message_generation
  )
find_package(Eigen REQUIRED)
find_package(PCL)
find_package(PkgConfig)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES
  TrackedObject.msg
  TrackedObjectList.msg
)

add_service_files(FILES
  ClassifyInstance.srv
  Classify.srv
  )

generate_messages(DEPENDENCIES
  geometry_msgs
  shape_msgs
  std_msgs
  )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime rospy
  )

###########
## Build ##
###########

## Check for what version of YAML we have
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DYAMLCPP_GT_0_5_0)
endif()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

## Declare a cpp executable
add_executable(simple_recognition src/SimpleRecognition.cpp)
add_executable(simple_data_collector src/SimpleDataCollector.cpp)
add_executable(simple_recognition_tester src/SimpleRecognitionTester.cpp)
add_executable(object_labeler src/ObjectLabeler.cpp)
add_executable(object_listener src/ObjectRecognitionListener.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(simple_recognition ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} yaml-cpp)
target_link_libraries(simple_data_collector ${catkin_LIBRARIES})
target_link_libraries(simple_recognition_tester ${catkin_LIBRARIES})
target_link_libraries(object_labeler ${catkin_LIBRARIES})
target_link_libraries(object_listener ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
add_dependencies(simple_recognition rail_manipulation_msgs_gencpp ${PROJECT_NAME}_generate_messages)
add_dependencies(simple_data_collector rail_manipulation_msgs_gencpp)
add_dependencies(simple_recognition_tester rail_manipulation_msgs_gencpp ${PROJECT_NAME}_generate_messages)
add_dependencies(object_labeler rail_manipulation_msgs_gencpp ${PROJECT_NAME}_generate_messages)
add_dependencies(object_listener rail_manipulation_msgs_gencpp ${PROJECT_NAME}_generate_messages)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS simple_recognition simple_data_collector simple_recognition_tester
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )
