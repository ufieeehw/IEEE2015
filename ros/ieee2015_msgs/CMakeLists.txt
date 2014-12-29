cmake_minimum_required(VERSION 2.8.3)
project(ieee2015_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  message_runtime
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Mecanum.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ieee2015_msgs
  CATKIN_DEPENDS message_runtime std_msgs rospy
)

## Build ##

include_directories(
  ${catkin_INCLUDE_DIRS}
)

