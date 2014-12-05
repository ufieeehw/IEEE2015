#/usr/bin/env python   

import rospy
import os
import argparse

#import all messages used to pass data through xMega Driver

from std_msgs.msg import Header, Float64, String
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion, Vector3, TwistStamped
from sensor_msgs.msg import Imu
from ieee2015_xmega_driver.msg import XMega_Message


rospy.init_node('xmega_codes', anonymous=True)

def debug_poll():

	pub = rospy.Publisher('robot/debug', String, queue_size=1)
	r = rospy.Rate(5) 

	count = 0
	while not rospy.is_shutdown() and count == 0:
		string = "debug"
		rospy.loginfo(string)
		pub.publish(string)
		r.sleep()
		count+=1

def imu_poll():

	pub = rospy.Publisher('robot/imu', String, queue_size=1)
	r = rospy.Rate(5) 

	count = 0
	while not rospy.is_shutdown() and count == 0:
		string = "imu"
		rospy.loginfo(string)
		pub.publish(string)
		r.sleep()
		count+=1

'''def step_motor_send():

	pub = rospy.Publisher('robot/desired_velocity', String, queue_size=1)
	r = rospy.Rate(5) 

	count = 0
	while not rospy.is_shutdown() and count == 0:
		string = "stepper motor"
		rospy.loginfo(string)
		pub.publish(string)
		r.sleep()
		count+=1
		'''

while True:
	debug_poll()
	imu_poll()
	#step_motor_send()
