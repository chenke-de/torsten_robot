cmake_minimum_required(VERSION 2.8.3)
project(torsten_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    TorstenState.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
)
