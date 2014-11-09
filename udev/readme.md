FTDI udev rule setup
====================

This enables the computer to recognize the Dynamixel servos (and eventually the XMega) as specific devices over USB, and place them at specific ports every time. That makes it really easy to deal with them.


# Usage

Run the following...
	cd ~/catkin_ws/src/udev
	chmod +x ./setup.sh
	sudo ./setup.sh

This will set up the udev rules. If you get a "file already exists" error, do

	rm /etc/udev/rules.d/99-ftdi.rules 

and then re-run setup.sh

If you run into issues, find Jacob

# Glossary

Dynamixel - The company that makes our servos

FTDI - A company that makes serial-USB converts that we make a lot of use of

udev - Linux rules you can set up so that a device doesn't automatically default to being at a port. A device will often automatically go to /dev/ttyUSB0, which is tough to work with if you have multiple devices and you serial interface requires a specific port to be chosen. This enables us to automatically put something it recognizes as a dynamixel at a port we make up, for example, /dev/dynamixel_tty