#/usr/bin/env python   

import os
import rospy
import time
import codecs
import sys
import readline

#import all messages used to pass data through xMega Driver

from std_msgs.msg import Header, Float64, String
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion, Vector3, TwistStamped, Twist
from sensor_msgs.msg import Imu
from ieee2015_xmega_driver.msg import XMega_Message

# create paths to use for type parsing

default_path_to_types = os.path.join('..', '..', '..', '..', 'xmega', 'types.h')
default_path_to_parse= os.path.join('..', '..', '..', '..', 'ros', 'ieee2015_xmega_driver', 'src', 'xmega_driver')
sys.path.insert(0, default_path_to_parse)
from parse_types import parse_types_file

global types_array # array to hold parsed types data structure
global just_types
global found_bool  # boolean to use in main loop for command compliance check

# <<-------------------------------------- Function Definitions ------------------------------------------------>>

class MyCompleter(object):  # Custom completer

    def __init__(self, options):
        self.options = sorted(options)

    def complete(self, text, state):
        if state == 0:  # on first trigger, build possible matches
            if text:  # cache matches (entries that start with entered text)
                self.matches = [s for s in self.options 
                                    if s and s.startswith(text)]
            else:  # no text entered, all matches possible
                self.matches = self.options[:]

        # return match indexed by state
        try: 
            return self.matches[state]
        except IndexError:
            return None

# ---------------------------------------------------------------------------------------------

def to_ascii(raw):

	# Convert HEX value to ascii character

	string = str(raw)
	asc = string.encode(encoding='UTF-8',errors='strict')
	return asc

# ---------------------------------------------------------------------------------------------

def types_parse():

	global types_array
	global just_types
	parsed_types = parse_types_file(default_path_to_types)				

	# size two dimensional array to be list length amount of rows with two collumns
	# index 0 is type name
	# index 1 is hex value

	types_array = [[0 for x in range(2)] for x in range(len(parsed_types))] 
	just_types = [0 for x in range(len(parsed_types))]

	for x in range(0, len(parsed_types)):			# as long as the dictionary has values									
		to_dict = parsed_types[x]					# convert list line to dictionary
		types_array[x][0] = to_dict['type_name']	# add types to array
		just_types[x] = to_dict['type_name']	# add types to array
		types_array[x][1] = to_dict['hex_name']		# add correlating ascii character to array

# ---------------------------------------------------------------------------------------------

def possible_types():

	global types_array

	# iterate through array to print possible type values to send
	x = 0
	for i in range(0, len(types_array)/4):
		for y in range(0, 4):
			print types_array[x][0] + " |",
			x = x+1
		print  ""

	print 
	print "<--- These are the possible types to send --->"
	print "<--- Press TAB to auto complete --->"
	print "<--- Enter 'help' for formatting help--->"

# ---------------------------------------------------------------------------------------------

def help():
	print
	print "<------------------------ FORMATTING HELP -------------------------"
	print
	print "<--- TYPE parameter-1 parameter-2 parameter-3 ... parameter-N --->"
	print
	print "<--- DEBUG_TYPE parameter"
	print "<--- STEP_MOTOR_TYPE parameter-1 ... parameter-6"
	print "<--- MOTOR_SPEED_TYPE parameter-1 ... parameter-6"
	print "<--- IMU_DATA_TYPE <-- Returns value of IMU data being publsihed from Xmega"
	print 
	print "<------------------------------------------------------------------"
# <----------------------------------- Subscriber Definitons ---------------------------------------------->

def imu_poll():

	sub = rospy.Subscriber('robot/imu', Imu,)
	r = rospy.Rate(5) 

	while not rospy.is_shutdown():
		rospy.loginfo("IMU recived %s", sub)
		break
	r.sleep()

# <----------------------------------- Publisher Definitons ---------------------------------------------->

def debug_poll(hex_value):

	global types_array

	pub = rospy.Publisher('robot/debug', String, queue_size=10)
	r = rospy.Rate(5) 

	msg = [hex_value]
	count = 0

	while not rospy.is_shutdown() and count != len(msg):
		for item in msg:

			converted = to_ascii(item)
			pub.publish(converted)
			rospy.loginfo("Debug polled %s", item)
			count = count + 1
		r.sleep()

# ---------------------------------------------------------------------------------------------

#def step_motor_send(a,b,c,d,e,f):

def motor_speed_send(a,b,c,d,e,f):

	global types_array

	pub = rospy.Publisher('robot/desired_velocity', TwistStamped, queue_size=10)
	r = rospy.Rate(5) 

	main = TwistStamped()
	stepper = Twist()
	stepper.linear.x = a
	stepper.linear.y = b
	stepper.linear.z = c
	stepper.angular.x = d
	stepper.angular.y = e
	stepper.angular.z = f

	main.twist = stepper
	now = rospy.get_rostime()
	main.header.stamp.secs = now.secs
	main.header.stamp.nsecs = now.nsecs
	main.header.frame_id = "robot/desired_velocity"
	
	pub.publish(main)
	rospy.loginfo("Stepper Motor sent %s", main)
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

completer = MyCompleter(just_types)
readline.set_completer(completer.complete)
readline.parse_and_bind('tab: complete')

# ----------------- Begin Main Loop ---------------------

while not rospy.is_shutdown():

	print 
	global types_array
	global found_bool
	found_bool = False	# reset value on every loop 

	# loop through types array until a matching type is found
	# if command matches type in array, coresponding function is called
	# if no type with the same name is found, error is printed

	send_type = raw_input("Command: ", )

	if len(send_type.split()) >= 2:

		split_array = send_type.split()

		for x in range(0, len(types_array)):

			if types_array[x][0] == split_array[0]:
				found_bool = True

				if split_array[0] == "DEBUG_TYPE":
					if len(split_array) != 2:
						print
						print "Improper debug type length"
						help()
					else:
						debug_poll(split_array[1])
				#if split_array[0] == "STEP_MOTOR_TYPE":
				if split_array[0] == "MOTOR_SPEED_TYPE":
					if len(split_array) != 7:
						print
						print "Improper step motor length"
						help()
					else:
						step_motor_send()
						motor_speed_send(
						split_array[1],
						split_array[2],
						split_array[3],
						split_array[4],
						split_array[5],
						split_array[6]
						)

		if found_bool == False:
			print "****TYPE NOT FOUND*****"
			possible_types()

	els:
		if send_type == "help":
			help()
		if send_type == "help":
			help()
		if send_type == "IMU_DATA_TYPE":
			imu_poll()
		else:
			print
			print "Only entered message type"
			print "Must include parameters"
			print "Please refer to formatting for sending messages"
