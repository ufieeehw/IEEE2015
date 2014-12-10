# IEEE 2015 Gazebo Simluation

## Installation

```sudo apt-get install ros-indigo-gazebo-ros-pkgs```

```echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/IEEE2015/ros/ieee2015_gazebo_sim/" >> ~/.bashrc```

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
* `/robot/jointX_position_controller/command std_msgs/Float64` Where x is a value between 1-3 for the 3 joints of the arm, sets the position of the joint to the radian value.  See use a little furthur down.

#### Unsimulated

* The mecanum drive is not simulated, instead the desired velocity is immediately achieved

## Controlling the Robot in Gazebo

* You should already have gazebo installed previously, you also need to install ros_control:
  ```sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers```

* To launch gazebo and set up the robot:
  `roslaunch ieee2015_gazebo_sim gazebo.launch`

  You should now see the robot standing on the starting position in Gazebo.

* To move the robot:
  Type the command ```rostopic pub /desired_velocity geometry_msgs/Twist``` then hit space, then tab twice, it should bring up a   list of x y z under Linear and Angular, use linear x y to move the robot, and angular z to turn it.
apt-get
* To move the arm:
  Base:
  ```rostopic pub -1 /robot/base_controller/command std_msgs/Float64 "data: 1.5"```
  "Bicep":
  ```rostopic pub -1 /robot/shoulder_controller/command std_msgs/Float64 "data: 1.5"```
  "Forearm":
  ```rostopic pub -1 /robot/elbow_controller/command std_msgs/Float64 "data: 1.5"```
  The "data" is a radian value for the arm position

* New feature! Move the arm without stupid commands!
  Do the previous stated setup procedure.
  Go to ```"Your workspace"/IEEE2015/ros/ieee2015_controller/src``` and run ```./arm_controller.py```
  Go to ```"Your workspace"/IEEE2015/ros/ieee2015_simulator/scripts``` and run ```./arm_simulator.py```
  Have fun clicking stuff instead of typing! Left side of the window is the front of the arm.

## FIX
- Joint1 rotates the whole base in the opposite direction
- The arm jitters a lot
- The mecanum wheels appear to have physical interaction with the world, causing jittering. Bad because they are not simulated, so they only interfere with the robot
- The model is improperly assembled
## Notes

In the misc folder there's a lot of reusable urdf stuff, but none of it is actually used. 
It contains urdf stuff for a Kinect, and some other mish mash. Look through it if you are trying to add stuff.
