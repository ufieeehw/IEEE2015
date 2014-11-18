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



# TODO
* Add a parser for the XMega-side communication protocol's definition header, to avoid making frequent changes. This should map the relationships of type names to ROS topics
* Add more failure tolerance - if there is a serial desync nobody knows what will happen

# Glossary

 * Poll Message - A message sent from ROS
 * Callback Function - A function that is called automatically when an event happens
 * Setup.py - A file that exposes a package to everything
