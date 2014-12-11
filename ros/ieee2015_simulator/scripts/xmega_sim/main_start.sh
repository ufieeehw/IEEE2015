#!/bin/bash

echo 
echo
echo "BEGIN FULL SIMULATION SETUP"
echo "<-------------------------------------------------------------->"
echo "Opened XMEGA SIMULATOR -->"
echo "Opened ROS SIMULATOR -->"
echo "Please begin xMega and Port Setup by entering password in new window"
echo "roslaunch will not begin until XMEGA SIMULATOR setup is complete"
echo 

gnome-terminal --title="XMEGA SIMULATOR" -e 'bash -c "sudo python xmega_communication_sim.py"' &
gnome-terminal --title="ROS SIMULATOR" -e 'bash -c "python ROS_send.py"' &
roslaunch ieee2015_xmega_driver xmega_driver.launch
