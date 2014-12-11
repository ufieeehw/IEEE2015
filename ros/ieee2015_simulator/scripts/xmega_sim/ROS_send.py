#/usr/bin/env python   

import os
import argparse
import rospy
import time
import codecs

#import all messages used to pass data through xMega Driver

from std_msgs.msg import Header, Float64, String
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion, Vector3, TwistStamped
from sensor_msgs.msg import Imu
from ieee2015_xmega_driver.msg import XMega_Message

# -------------------------- Function Definitions -----------------------------------------------

def to_ascii(raw):
	string = str(raw)
	asc = string.encode(encoding='UTF-8',errors='strict')
	return asc


# -------------------------- Subscriber Definitons ----------------------------------


def imu_poll():

	sub = rospy.Subscriber('robot/imu', Imu)
	r = rospy.Rate(5) 

	while not rospy.is_shutdown():
		msg = [0xEF]
		for item in msg:
			converted = to_ascii(item)
			pub.publish(converted)
			rospy.loginfo("imu polled %s", item)
		r.sleep()


# -------------------------- Publisher Definitons ------------------------------------


def debug_poll():

	pub = rospy.Publisher('robot/debug', String, queue_size=10)
	r = rospy.Rate(5) 

	msg = [0x40,0x00]
	count = 0

	while not rospy.is_shutdown() and count != len(msg):

		for item in msg:
			converted = to_ascii(item)
			pub.publish(converted)
			rospy.loginfo("Debug polled %s", item)
			count = count + 1
		r.sleep()


def step_motor_send():

	pub = rospy.Publisher('robot/desired_velocity', String, queue_size=1)
	r = rospy.Rate(5) 

	msg = [0x80,0x00,0x01]
	count = 0

	while not rospy.is_shutdown() and count != len(msg):
		for item in msg:
			converted = to_ascii(item)
			pub.publish(converted)
			rospy.loginfo("Stepper Motor sent %s", item)
			count = count + 1
		r.sleep()


# ------------------------------- End of Function Definitions -----------------------------------------

print "<------------------------------------------------------------------->"
print "ROS side simulation"
print 

print "st --> stepper motor send"
print "db --> debug send"

rospy.init_node('xmega_codes', anonymous=True)

# ------------------------------------ Begin Main Loop ------------------------------------------------

while not rospy.is_shutdown():

	send_type = raw_input("Command: ", )
	if send_type == "st":
		step_motor_send()
	if send_type == "db":
		debug_poll()

	


