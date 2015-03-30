cmake_minimum_required(VERSION 2.8.3)
project(ieee2015_tf_broadcaster)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_msgs
  geometry_msgs
  message_generation
  rospy
  std_msgs
  tf
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieee2014_tf_broadcaster
CATKIN_DEPENDS dynamixel_msgs geometry_msgs rospy std_msgs tf message_runtime
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

