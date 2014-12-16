#/usr/bin/env python   

import os
import rospy
import time
import codecs
import sys

#import all messages used to pass data through xMega Driver

from std_msgs.msg import Header, Float64, String
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion, Vector3, TwistStamped
from sensor_msgs.msg import Imu
from ieee2015_xmega_driver.msg import XMega_Message

# create paths to use for type parsing

default_path_to_types = os.path.join('..', '..', '..', '..', 'xmega', 'types.h')
default_path_to_parse= os.path.join('..', '..', '..', '..', 'ros', 'ieee2015_xmega_driver', 'src', 'xmega_driver')
sys.path.insert(0, default_path_to_parse)
from parse_types import parse_types_file

global types_array # array to hold parsed types data structure
global found_bool  # boolean to use in main loop for command compliance check


# <<-------------------------------------- Function Definitions ------------------------------------------------>>

def to_ascii(raw):

	# Convert HEX value to ascii character

	string = str(raw)
	asc = string.encode(encoding='UTF-8',errors='strict')
	return asc

# ---------------------------------------------------------------------------------------------

def types_parse():

	global types_array
	parsed_types = parse_types_file(default_path_to_types)				

	# size two dimensional array to be list length amount of rows with two collumns
	# index 0 is type name
	# index 1 is hex value

	types_array = [[0 for x in range(2)] for x in range(len(parsed_types))] 	

	for x in range(0, len(parsed_types)):			# as long as the dictionary has values									
		to_dict = parsed_types[x]					# convert list line to dictionary
		types_array[x][0] = to_dict['type_name']	# add types to array
		types_array[x][1] = to_dict['hex_name']		# add correlating ascii character to array

# ---------------------------------------------------------------------------------------------

def possible_types():

	global types_array

	print "<--- These are the possible types to send --->"
	print

	# iterate through array to print possible type values to send

	for x in range(0, len(types_array)):
		print types_array[x][0]

# <----------------------------------- Subscriber Definitons ---------------------------------------------->

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

# <----------------------------------- Publisher Definitons ---------------------------------------------->

def debug_poll(index):
	global types_array

	pub = rospy.Publisher('robot/debug', String, queue_size=1)
	r = rospy.Rate(5) 

	msg = [types_array[index][1], 0]
	count = 0

	while not rospy.is_shutdown() and count != len(msg):

		for item in msg:
			pub.publish(item)
			rospy.loginfo("Debug polled %s", item)
			count = count + 1
		r.sleep()

# ---------------------------------------------------------------------------------------------

def step_motor_send():

	pub = rospy.Publisher('robot/desired_velocity', TwistStamped, queue_size=1)
	r = rospy.Rate(5) 

	msg = [0, 0]

	while not rospy.is_shutdown():
		pub.publish(msg)
		rospy.loginfo("Stepper Motor sent %s", item)
		r.sleep()


# <<----------------------------------- End of Function Definitions ------------------------------------------>>

print 
print "<------------------------------------------------------------------->"
print
print "<--- ROS SIDE SIMULATION"
print 

types_parse()
possible_types()

rospy.init_node('xmega_codes', anonymous=True)


# ----------------- Begin Main Loop ---------------------

while not rospy.is_shutdown():

	print 
	global types_array
	global found_bool

	found_bool = False	# reset value on every loop 
	
	send_type = raw_input("Command: ", )

	# loop through types array until a matching type is found
	# if command matches type in array, coresponding function is called
	# if no type with the same name is found, error is printed

	for x in range(0, len(types_array)):
		if types_array[x][0] == send_type:
			found_bool = True
			if send_type == "DEBUG_TYPE":
				debug_poll(x)
			if send_type == "STEP_MOTOR_TYPE":
				step_motor_send()

	if found_bool == False:
		print
		print "TYPE NOT FOUND"
		print 
		possible_types()
		


	


