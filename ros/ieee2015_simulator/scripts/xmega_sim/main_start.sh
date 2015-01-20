#!/bin/bash
echo 
echo
echo "BEGIN FULL SIMULATION SETUP"
echo "<-------------------------------------------------------------->"
gnome-terminal --title="XMEGA SIMULATOR" -e 'bash -c "sudo python xmega_communication_sim.py"' &
echo "Opened XMEGA SIMULATOR -->"
echo "roslaunch will not begin until XMEGA SIMULATOR setup is complete" &
gnome-terminal --title="ROS SIMULATOR" -e 'bash -c "roslaunch ieee2015_xmega_driver xmega_driver.launch"'
echo "Opened ROSLAUNCH - XMEGA_DRIVER -->"
sleep 3
gnome-terminal --title="ROS SIMULATOR" --geometry="70x30" -e 'bash -c "python ROS_send.py"' &
echo "Opened ROS SIMULATOR -->"
echo 
echo "NOW RUNNING FULL SIMULATION"
echo 
