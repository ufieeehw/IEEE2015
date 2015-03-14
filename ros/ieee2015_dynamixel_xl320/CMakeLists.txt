cmake_minimum_required(VERSION 2.8.3)
project(end_effector_servos)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
  FILES
  Num.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(end_effector_sub nodes/end_effector_sub.cpp)
target_link_libraries(end_effector_sub ${catkin_LIBRARIES})
add_dependencies(end_effector_sub end_effector_servos_generate_messages_cpp)

add_executable(end_effector_pub nodes/end_effector_pub.cpp)
target_link_libraries(end_effector_pub ${catkin_LIBRARIES})
add_dependencies(end_effector_pub end_effector_servos_generate_messages_cpp)