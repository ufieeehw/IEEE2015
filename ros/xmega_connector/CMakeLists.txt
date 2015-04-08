cmake_minimum_required(VERSION 2.8.3)
project(xmega_connector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
)


# catkin_python_setup()


add_message_files(
  FILES
  XMEGAPacket.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Echo.srv
  SetWheelSpeeds.srv
  GetOdometry.srv
  GetColor.srv
  GetHeading.srv
  GetMotion.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xmega_connector
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
