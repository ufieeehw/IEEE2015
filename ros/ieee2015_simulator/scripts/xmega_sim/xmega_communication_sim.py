# MUST RUN AS SUDO UNLESS SHELL RUNS AS SUDO AUTOMATICALLY
# must install socat to operate
# www.dest-unreach.org/socat/
# or 
# sudo apt-get install socat

import subprocess
import sys
import serial
import time
import os
import codecs

default_path_to_types = os.path.join('..', '..', '..', '..', 'xmega', 'types.h')
default_path_to_parse= os.path.join('..', '..', '..', '..', 'ros', 'ieee2015_xmega_driver', 'src', 'xmega_driver')
sys.path.insert(0, default_path_to_parse)
from parse_types import parse_types_file

print 
print "XMEGA SIMULATION"

proper_start = False # boolean used to check for startup byte
master_shutdown = True
read_ser = 0
read_ser_n = 0

types_array = []

subprocess.call(["chmod", "u+x", "./com_ports_on.sh"])
subprocess.Popen('./com_ports_on.sh')


# <--------------------------------Function Definitions--------------------------------->

# Main initialization function 

def init():

	global read_ser
	global read_ser_n
	count = 0;
	name = False

	# run loop to allow time for OS to recognize xmega_tty and ttyS30
	# If loop runs 200 times it is assumed there is an error with setup

	print "-----------------------------------------------"
	while name == False and count != 200:
		time.sleep(.5)
		print "Configuring"
		name = os.path.exists('/dev/ttyS30')
		count+=1

	if name == True:
		print "-----------------------------------------------"
		print "Socat COM simulation succesful"
		print "Ports will be closed when this window is closed"
		print "The proper PTS ports are now displayed in the popup terminal"
		print "-----------------------------------------------"
	elif name == False:
		print
		print "Could not create tty ports"
		print
		print "Please run progam as root to continue"
		print
		sys.exit(0)

	subprocess.call(["chmod", "666", "/dev/xmega_tty"]) #change permission to allow ros access

	linked_path = os.path.realpath("/dev/xmega_tty")
	sliced_path = linked_path[9:] # returns pts number the tty port is linked to

	linked_path_two = os.path.realpath("/dev/ttyS30")
	sliced_path_two = linked_path_two[9:] # returns pts number the tty port is linked to

	convert_two = '/dev/pts/' + str(sliced_path)
	convert_one = '/dev/pts/' + str(sliced_path_two)

	print 'Waiting for signal from xmega driver at ' , convert_one
	print 'Waiting for signal from xmega driver at ' , convert_two
	print 

	read_ser = serial.Serial(convert_one)
	read_ser_n = serial.Serial(convert_two)

# <--------------------------------------------------------------------------------------->

# parses types and converts to hex for interpretation as the xMega

def types_parse():

	global types_array
	parsed_types = parse_types_file(default_path_to_types)				

	# size two dimensional array to be list length amount of rows with two collumns
	# index 0 is type name
	# index 1 is hex value

	types_array = [[0 for x in range(2)] for x in range(len(parsed_types))] 	

	for x in range(0, len(parsed_types)):			# as long as the dictionary has values									
		to_dict = parsed_types[x]					# convert list line to dictionary
		hexed = hex_determine(to_dict['hex_name'])
		types_array[x][0] = to_dict['type_name']	# add types to array
		types_array[x][1] = hexed					# add correlating ascii character to array

# <--------------------------------------------------------------------------------------->

# function to convert character into hex value

def hex_determine(message):

	if type(message) is str:
		convert = hex(ord((message)))
		return convert
	if type(message) is int:
		convert = hex(message)
		return convert

# <--------------------------------------------------------------------------------------->

# function to output type and value of byte recieved

def type_determine_out(hex_value):
	
	global types_array
	type_bool = False

	for x in range(0,len(types_array)):
		if hex_value == types_array[x][1]:
			print  "Incoming: " + types_array[x][0] + " --- " + types_array[x][1]
			if types_array[x][0] == "IMU_NOTIFY_TYPE":
				type_bool = True

	if(type_bool == False):
		print  "Incoming: UNKNOWN VALUE --- " + hex_value

# <--------------------------------------------------------------------------------------->

# Want to fill with all possible step motor returns 

def step_motor_call(hex_value):
	return '0x00'

# <--------------------------------------------------------------------------------------->

# Want to fill with all possible debug returns 

def imu_type_call():
	for x in range(0,7):
		write_to_ros('0xEF')

# <--------------------------------------------------------------------------------------->

def startup_loop(to_hex):

	global proper_start 

	if to_hex == '0x2':
		print "RECIEVED STARTUP BYTE"
		proper_start = True
	else:
		print "ERROR - Did not recieve startup byte"
		master_shutdown = False

# <--------------------------------------------------------------------------------------->

# write values to ROS depending on recieved values - In hex or as char???

def write_to_ros(hexed):
	read_ser.write(hexed)

# <--------------------------------------------------------------------------------------->

# Read and Convert values being sent from Ros to the xMega

def read_from_ros():

	# Loop through at least one time to check that startup is first byte recieved
	
	global proper_start 

	Message = read_ser.read() 					# read one byte
	to_hex = hex_determine(Message)				# convert byte to hex

	if proper_start == False:
		startup_loop(to_hex)

	hex_value = type_determine_out(to_hex)		# what is the byte telling us to do?
	return to_hex								# return value to main loop


# <----------------------------- End of Functions Section -------------------------------->

init()
types_parse()

# <-------------------------------------MAIN LOOP----------------------------------------->

while master_shutdown:

	data_returned = False
	initial_value = read_from_ros()



