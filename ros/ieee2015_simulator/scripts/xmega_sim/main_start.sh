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

echo "Enter [y/yes] when ready to continue setup and launch Xmega Driver"
echo 

gnome-terminal --title="XMEGA SIMULATOR" -e 'bash -c "sudo python xmega_communication_sim.py;exec $SHELL"' &
gnome-terminal --title="ROS SIMULATOR" -e 'bash -c "python ROS_send.py;exec $SHELL"' &

read cont

if [[ ($cont == 'y' ) || ( $cont == 'yes' ) ]]; then
	gnome-terminal --title="XMEGA DRIVER" -e 'bash -c "roslaunch ieee2015_xmega_driver xmega_driver.launch"' 
	echo 
	echo "GO GO GO"
	echo
fi
