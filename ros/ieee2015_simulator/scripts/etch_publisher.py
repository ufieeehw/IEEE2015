#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from ieee2015_end_effector_servos.msg import high_level
import numpy as np
import time

rospy.init_node('pub-test', anonymous=True)

pub = rospy.Publisher('end_efffector_angles_sub', high_level)

pi = np.pi

array = [[0,	   0],
		[pi*3/4,   0],
		[pi*5/12,  0],
		[pi*5/12, pi],
		[0,		  pi],
		[pi*3/2,  pi],
		[pi,	  pi],
		[pi,	pi*1/2],
		[pi*3/2,pi*1/2],
		[pi,	pi*1/2],
		[pi,		0],
		[pi*9/4,	0],
		[pi*7/4,	0],
		[pi*7/4,pi*1/2],
		[pi*9/4,pi*1/2],
		[pi*7/4,pi*1/2],
		[pi*7/4,  pi],
		[pi*3,	  pi],
		[pi*5/2,  pi],
		[pi*5/2,pi*1/2],
		[pi*3,	pi*1/2],
		[pi*5/2,pi*1/2],
		[pi*5/2,	0],
		[pi*3,		0]]

for i in range(len(array)):
	pub.publish(high_level(large_radians=array[i][0]/5, small_radians=array[i][1]/5))
	print array[i][0],array[i][1]
	time.sleep(.25)


rospy.spin()