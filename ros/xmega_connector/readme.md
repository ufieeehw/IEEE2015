xmega_connector
===============

IEEE2015 Xmega Connector, based in last year's code


# Usage
    
First you have to set up UDEV rules, or xmega_tty won't be recognized. To do this, you must go to the IEEE2015/udev folder and run
    sudo bash ./setup.sh

To run the connector, do  

    roslaunch xmega_connector xmega_connector.launch

To send a command to the motors, do
    
    rosservice call /xmega_connector/set_wheel_speeds "wheel1: 0 wheel2: 0 wheel3: 0 wheel4: 0" 

To set the wheel speeds in rad/s
(When you are typing it, do tab-completion to get the "wheel1...wheel4" string to autofill)


# Potential Issues

"Could not find port xmega_tty" -> Make sure you set up udev rules
"Could not open port xmega_tty" -> Make sure you set up udev rules, AND that you have permissions for the port
For permissions, try (I apologize to all that is good and right in this world for saying this):

    sudo chmod 777 /dev/xmega_tty
    sudo chmod 777 /dev/ttyUSB*

Otherwise, if you already know how permissions work in Ubuntu, chmod them to something appropriate and reasonable.
