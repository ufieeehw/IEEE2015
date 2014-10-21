Xmega Driver
============

This is the ros-side communication protocol for interfacing with the XMega microcontroller.

# TODO
* Add a parser for the XMega-side communication protocol's definition header, to avoid making frequent changes. This should map the relationships of type names to ROS topics
* Code review the whole XMega set and start to converge at a simple, unified solution

# Needed on XMega Side
* Serial-Triggered reset
* Driver requirements need to be solidified before we make serious changes