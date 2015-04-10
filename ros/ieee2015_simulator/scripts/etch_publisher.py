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
		[pi*3/4,   0],#right
		[pi*5/12,  0],#left
		[pi*5/12, pi],#down
		[0,		  pi],#left
		[pi*3/2+.5,  pi],#right bottom E ###########
		[pi,	  pi],#left
		[pi,	pi*1/2],#up, first half E
		[pi*3/2+.5,pi*1/2],#out first leg ##########
		[pi,	pi*1/2],#in
		[pi,		0],	#top of E
		[pi*9/4+.5,	0], #top of second E ########
		[pi*7/4,	0],	#back
		[pi*7/4,pi*1/2],#down half 2 E
		[pi*9/4+.5,pi*1/2],#out leg ################
		[pi*7/4,pi*1/2],#back in
		[pi*7/4,  pi],  #bottom E
		[pi*3+.5,	  pi],	#right to bottom 3 E ####
		[pi*5/2,  pi],	#back
		[pi*5/2,pi*1/2],#up half 3 E
		[pi*3+.5,	pi*1/2],#out leg ################
		[pi*5/2,pi*1/2],#back in
		[pi*5/2,	0],	#up top E
		[pi*3+.5,		0]]	#END     ################

for i in range(len(array)):
	pub.publish(high_level(large_radians=array[i][0]/5, small_radians=array[i][1]/5))
	print array[i][0],array[i][1]
	time.sleep(.25)


rospy.spin()