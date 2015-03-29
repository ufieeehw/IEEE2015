cmake_minimum_required(VERSION 2.8.3)
project(ieee2015_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  message_runtime
)

catkin_python_setup()

add_message_files(
    FILES
    Float_List.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieee2015_simulator
 CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)