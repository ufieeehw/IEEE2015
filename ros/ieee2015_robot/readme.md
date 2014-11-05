IEEE 2015 Robot Description
===========================


# Purpose
This folder contains the package that fully describes the IEEE robot for 2015, including joints, cameras and arm behavior. This package is used by the ieee2015_gazebo_sim package to generate the robot.

# Contains
* Solidworks models of the chassis and arm
* Sensor descriptions
* Ros_Control package for robot control

# Not Yet Implemented
* Actual involvement of the Mecanum Wheels (Right now we ignore the wheels and move in the direction of desired velocity)
* We should include inertial qualities)


# Necessary packages
* apt-get ros-indigo-gazebo-ros-control ros-indigo-ros-control ros-indigo-ros-controllers 

