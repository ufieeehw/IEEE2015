# IEEE 2015 Gazebo Simluation

## Installation

```sudo apt-get install ros-indigo-gazebo-ros-pkgs

echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/IEEE2015/ros/ieee2015_gazebo_sim/" >> ~/.bashrc```

Where ~/catkin_ws is the path to your ros workspace. (If you followed the first few ROS tutorials, this should be the exact path)

You should not need to install any other packages to make the simulator work. If you have any errors, see Nikita or Jacob, it is likely  problem with the sim implementation.

## Usage

Do 
```roslaunch ieee2015_gazebo_sim gazebo.launch```
to start the simulation. You should see the robot (with some orange and blue coloring) flat on top of a black/white course.

### The Vehicle base

#### Important topics
* `desired_velocity` sets a Twist velocity command in the robot frame
* `odom` publishes simulated robot odometry
* `robot/camera1/image` publishes images from the forward camera, view it using ```rosrun image_view image_view image:=/robot/camera1/image```

We don't yet have a correct position publisher

#### Unsimulated

* The mecanum drive is not simulated, instead the desired velocity is immediately achieved


### The Arm
Coming soon!

#### Joint Control Topics
Coming Soon!

## Notes

In the misc folder there's a lot of reusable urdf stuff, but none of it is actually used. 
It contains urdf stuff for a Kinect, and some other mish mash. Look through it if you are trying to add stuff.