#!/usr/bin/python
from __future__ import division
## Math
import numpy as np
## Ros
import rospy
import tf.transformations as tf_trans
import tf
## Ros Msgs

class SCARA_Controller(object):
	'''APPLIES ONLY TO 2 DOF ARM, PROOF OF CONCEPT'''
    def __init__(self):
        rospy.init_node('SCARA_controller')
        self.length_1 = self.length_2 = 20
        self.base = np.array([0, 0], np.float32)

        
    def solve_angles(self, pt):
    	'''2DOF has a closed form solution, this method only slightly extends to higher DOF, where there is not a closed form'''
    	x, y = pt
    	base_angle = np.arcctan(y/x) - arccos(np.sqrt(x**2 + y**2) / 2*self.length_1)
    	abs_joint_angle = np.arctan( (y - (self.length_1 * np.sin(base_angle))) / (x - (self.length_1 * np.cos(base_angle))) )
    	joint_angle = abs_joint_angle - base_angle

		normalize = lambda o: o % (2 * math.pi) - math.pi    	
    	return map(normalize, (base_angle, joint_angle))


if __name__ == '__main__':
	tests = map(
		lamba o: np.array(o, np.float32), 
		[0.5,0.5],
		[21,21],
		[500,500],
		[12,13],
		[4,9],
	)
	SCARA = SCARA_Controller()

	for test in tests:
		print SCARA.solve_angles(tests)
