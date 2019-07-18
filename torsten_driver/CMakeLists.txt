cmake_minimum_required(VERSION 2.8.3)
project(torsten_driver)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rosout
  nav_msgs
  tf
  rospy
  std_msgs
  message_generation
  move_base_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
# services
add_service_files(
    DIRECTORY srv
    FILES
    setSound.srv
    setBolts.srv
    setBit.srv
    boltsState.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES torsten_driver
  CATKIN_DEPENDS roscpp sensor_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include/torsten_driver)

## Declare a C++ library
 add_library(torsten_driver
   include/torsten_driver/torsten_driver.cpp
 )

## Add cmake target dependencies of the executable
if(nav_msgs_EXPORTED_TARGETS)
  add_dependencies(torsten_driver ${nav_msgs_EXPORTED_TARGETS})
endif()

add_dependencies(torsten_driver ${torsten_driver_EXPORTED_TARGETS})
add_dependencies(torsten_driver torsten_driver_generate_messages)

## Declare a C++ executable
add_executable(torsten_driver_node src/torsten_driver_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(torsten_driver_node
  ${catkin_LIBRARIES}
  torsten_driver
  pcan
)

