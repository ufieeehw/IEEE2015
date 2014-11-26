# MUST RUN AS SUDO 
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
print "Xmega simulation setup..."
print 
print "The proper PTS ports are now displayed in the popup terminal"

# subprocess to change permission and open pseudo TTy Ports
subprocess.call(["chmod", "u+x", "./com_ports_on.sh"])
subprocess.Popen(["xterm", "-e", "./com_ports_on.sh"])

time.sleep(.5) # delay to allow time for chmod to recognize xmega_tty

subprocess.call(["chmod", "666", "/dev/xmega_tty"]) #change permission to allow ros access

linked_path = os.path.realpath("/dev/xmega_tty")
sliced_path = linked_path[9:] # returns pts number the tty port is linked to

linked_path_two = os.path.realpath("/dev/ttyS30")
sliced_path_two = linked_path_two[9:] # returns pts number the tty port is linked to

convert_one = '/dev/pts/' + str(sliced_path_two)
convert_two = '/dev/pts/' + str(sliced_path)

print 'Waiting for signal from xmega driver at ' , convert_two
print 'Waiting for signal from xmega driver at ' , convert_one

read_ser = serial.Serial(convert_one)
read_ser_n = serial.Serial(convert_two)

first_types = ['0x5e','0xC0','0x43','0x80']
second_types = ['0xC0','0xC0']
third_types = ['0x01','0x02','0x03','0x04']

proper_start = False # boolean used to check for startup byte


# function to convert character into hex value
def hex_determine(message):
	convert = hex(ord((message)))
	return convert


# function to output type and value of byte recieved
def type_determine_out(hex_value):

	allowed_values = ['0x2', '0x3']
	trap = False

	if hex_value == '0x2':
		print "Startup Byte -", hex_value
		trap = True
	if hex_value == '0x3':
		print "Keep Alive Byte -", hex_value
		trap = True

	if trap == False:
		for x in range(0, len(allowed_values)):
			if hex_value != allowed_values[x]:
				print "Unknown Value - ", hex_value


def write_to_ros(hexx):
	read_ser.write(hexx)

def read_from_ros():

	global proper_start

	Message = read_ser.read(1)
	to_hex = hex_determine(Message)
	type_determine_out(to_hex)

	# Loop through one time to check that startup is first byte recieved
	if proper_start == False:
		if to_hex == '0x2':
			print "RECIEVED STARTUP BYTE"
			proper_start = True
		else:
			print "ERROR - Did not recieve startup byte"
			sys.exit() # exit on error

while 1:

	read_from_ros()





