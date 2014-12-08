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

print 
print "XMEGA SIMULATION"

proper_start = False # boolean used to check for startup byte
master_shutdown = True
read_ser = 0
read_ser_n = 0


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
		print "Please run progam as sudo to continue"
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
	print 'Can now start ROS Launch from main terminal'

	read_ser = serial.Serial(convert_one)
	read_ser_n = serial.Serial(convert_two)


# <--------------------------------------------------------------------------------------->

# function to convert character into hex value

def hex_determine(message):
	convert = hex(ord((message)))
	return convert

# <--------------------------------------------------------------------------------------->

# function to output type and value of byte recieved

def type_determine_out(hex_value):

	allowed_values = ['0x01', '0x2', '0x3', '0x04']
	trap = False

	if hex_value == '0x1':
		print "Kill Byte -", hex_value
		print "xMega simulation terminated"
		master_shutdown = False
		trap = True
	if hex_value == '0x2':
		print "Startup Byte -", hex_value
		trap = True
	if hex_value == '0x3':
		print "Keep Alive Byte -", hex_value
		trap = True
	if hex_value == '0x4':
		print "Poll IMU Byte -", hex_value
		trap = True

	if trap == False:
		for x in range(0, len(allowed_values)):
			if hex_value != allowed_values[x]:
				print "Unknown Value - ", hex_value

# <--------------------------------------------------------------------------------------->

# Want to fill with all possible step motor returns 

def step_motor_call(hex_value):
	return '0x00'

# <--------------------------------------------------------------------------------------->

# Want to fill with all possible debug returns 

def debug_type_call(hex_value):
	return '0x00'

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

def write_to_ros(hexx):
	read_ser_n.write(hexx)

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


# <-------------------------------------MAIN LOOP----------------------------------------->

while master_shutdown:

	data_returned = False
	initial_value = read_from_ros()


	# recieived keep alive byte and listening to next byte
	if (initial_value == '0x3'):
		second_value = read_from_ros()
		to_decimal = int(second_value, 16)

		# recieived call to debug type
		if(to_decimal >= 64 and to_decimal <= 111 ):
			to_send = debug_type_call(second_value)
			write_to_ros(to_send)
			data_returned = True

			'''

			Am going to work this in to somehow get real debug types

			From xMega OS

			 //send outgoing message
  			Message out = get_msg(DEBUG_TYPE, 1);
 			out.data[0] = *m.data;
  			return queue_push(out, OUT_QUEUE);

			'''

		# recieved call to stepper motor
		if(to_decimal >= 128 and to_decimal <= 175 ):
			to_send = step_motor_call(second_value)
			print "Call to Stepmotor with value", second_value
			data_returned = False

	# recieved call to poll the IMU for data
	elif (initial_value == '0x4'):
		# Need to discuss return values for IMU polling in simulation
		print "Polled the IMU and returned value"
		write_to_ros("0xEF")


	if(data_returned == False):
		print "No Data Returned"







