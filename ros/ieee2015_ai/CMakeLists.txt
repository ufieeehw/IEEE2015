cmake_minimum_required(VERSION 2.8.3)
project(ieee2015_ai)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)
catkin_python_setup()

install(PROGRAMS
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieee2015_mission_control
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieee2015_AI
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
