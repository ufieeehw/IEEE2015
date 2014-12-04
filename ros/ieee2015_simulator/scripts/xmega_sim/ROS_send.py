#/usr/bin/env python   

import rospy
import os
import argparse


import roslib; roslib.load_manifest('ieee2015_xmega_driver')
import xmega_driver
from ieee2015_xmega_driver.msg import XMega_Message

#import all messages used to pass data through xMega Driver

from std_msgs.msg import Header, Float64, String
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion, Vector3, TwistStamped
from sensor_msgs.msg import Imu
from ieee2015_xmega_driver.msg import XMega_Message

def talk(self):

	rospy.init_node('xmega_codes', anonymous=True)
	pub = rospy.Publisher('robot/debug', String, queue_size=10)
	r = rospy.Rate(2) 

	while not rospy.is_shutdown():
		self.serial_proxy.add_message('ros_debug', "Test")
		rospy.loginfo(string)
		pub.publish(string)
		r.sleep()

if __name__ == '__main__':
	try:
		talk('ros_debug')
	except rospy.ROSInterruptException: pass