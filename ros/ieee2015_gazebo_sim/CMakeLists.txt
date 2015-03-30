cmake_minimum_required(VERSION 2.8.3)
project(ieee2015_gazebo_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
)

catkin_package(
  DEPENDS gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

