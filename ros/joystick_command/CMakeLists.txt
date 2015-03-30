cmake_minimum_required(VERSION 2.8.3)
project(joystick_command)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joystick_command
  CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
