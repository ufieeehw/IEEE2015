Xmega Driver
============

This is the ros-side communication protocol for interfacing with the XMega microcontroller.

# Usage

## Running in ROS

```roslaunch ieee2015_xmega_driver xmega_driver.launch```

## Adding things
* Changing the message lengths or message type names in types.h will automatically change things here
* Adding a function and binding it to a message type in the xmega_driver file will handle messages from the xmega
* Stand by for a better method of adding rostopics for the xmega to listen to

# Simulating
    Try socat to fake a serial port, 
    ```sudo socat PTY,link=/dev/ttyUSB0 PTY,link=/dev/COM1
    To fix permissions issues, do sudo chmod 666 /dev/ttyUSB0
## Necessary Simulation Elements
    - Need to test that incoming messages are of the appropriate length
    - Need to test that we're not getting garbage messages, or that any incorrect messages are being transferred
    - Need to test that the start message is sent before anything else
    - Need to be able to log the messages the simulator recieves in plain english so that we can understand where errors pop up
    
# TODO
* Add a parser for the XMega-side communication protocol's definition header, to avoid making frequent changes. This should map the relationships of type names to ROS topics
* Add more failure tolerance - if there is a serial desync nobody knows what will happen

# Glossary

 * Poll Message - A message sent from ROS
 * Callback Function - A function that is called automatically when an event happens
 * Setup.py - A file that exposes a package to everything
