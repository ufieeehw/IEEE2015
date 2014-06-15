Updated rubix identification algorithm
Program will:
 -Get picture
 -Filter colors
 -Map contours to RotatedRect's
 -Use rectangles to id center and angle

Ideally it will report back the center (offset from image center) and angle 
	(in radians) of a single Rubix cube in the picture. In debug mode,
	it will also display a line starting at the center and following the
	angle. Algorithm struggles with colored (Not black and white) noise 
	due to the filter method I'm using.

TODO:
Plugin to ROS.
Improve filter precision.
Increase preformance with missing tiles.

Usage:
./line2 <filename>
Debug mode can be enabled by recompiling at a different debug level (ie 3).

