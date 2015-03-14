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
  end_effector_servos
)

catkin_package(
    LIBRARIES xl320_library
    CATKIN_DEPENDS std_msgs
    DEPENDS include)


set(YOUR_LIB_SOURCES
    src/communications.cpp
    src/crc.cpp
    src/dxl_hal.cpp
    src/high_level_commands.cpp
    src/packet_utils.cpp)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(end_effector_sub nodes/end_effector_sub.cpp)
add_executable(end_effector_pub nodes/end_effector_pub.cpp)
add_library(xl320_library ${YOUR_LIB_SOURCES})
target_link_libraries(end_effector_sub ${catkin_LIBRARIES} xl320_library)

target_link_libraries(end_effector_pub ${catkin_LIBRARIES} xl320_library)
add_dependencies(end_effector_sub end_effector_servos_generate_messages_cpp)

add_dependencies(end_effector_pub end_effector_servos_generate_messages_cpp)
add_dependencies(end_effector_sub src/communications.cpp src/crc.cpp src/dxl_hal.cpp src/high_level_commands.cpp src/packet_utils.cpp)

install(TARGETS xl320_library 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )