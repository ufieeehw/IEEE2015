Arm Dynamixel Driver
================

This is the package that 

# Installing

Set up udev rules (So the computer recognizes the servos over USB)

	cd ~/catkin_ws/src/udev
	chmod +x ./setup.sh
	sudo ./setup.sh
This will set up the udev rules. If you get a "file already exists" error, do

	rm /etc/udev/rules.d/99-ftdi.rules 

and then re-run setup.sh


# Usage

To run it, do

	roslaunch ieee2015_servo_controller controller_manager.launch

	roslaunch ieee2015_servo_controller initalize_servos.launch


Then try

```rostopic pub /shoulder_controller/command std_msgs/Float64 "data: 1.6" ```

To move the physical 

# Naming

Base - Rotates whole arm about Z axis
Shoulder - Rotates the first link of the arm around the arm y-axis
Elbow - Rotates the second link of the arm around the arm y-axis
End-Effector - Rotates the end-effector (The "hand" of the arm)

# Glossary

Servo - A nice motor that lets us send a target angle or target velocity, and it figures out how to attain it

Dynamixel - The company that makes our servos

FTDI - A company that makes serial-USB converts that we make a lot of use of

udev - Linux rules you can set up so that a device doesn't automatically default to being at a port. A device will often automatically go to /dev/ttyUSB0, which is tough to work with if you have multiple devices and you serial interface requires a specific port to be chosen. This enables us to automatically put something it recognizes as a dynamixel at a port we make up, for example, /dev/dynamixel_tty