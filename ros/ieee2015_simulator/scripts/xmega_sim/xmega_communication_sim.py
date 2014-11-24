# MUST RUN AS SUDO 
# must install socat to operate
# www.dest-unreach.org/socat/
# or 
# sudo apt-get install socat

import subprocess
import sys
import serial
import time

print 
print "Xmega simulation setup..."
print 
print "The proper PTS ports are now displayed in the popup terminal"
print "Locate the port numbers and enter them now"

# subprocess to change permission and open pseudo TTy Ports
subprocess.call(["chmod", "u+x", "./com_ports_on.sh"])
subprocess.Popen(["xterm", "-e", "./com_ports_on.sh"])

time.sleep(.5) # delay to allow time for chmod to recognize xmega_tty

subprocess.call(["chmod", "666", "/dev/xmega_tty"]) #change permission to allow ros access

linked_path = os.path.realpath("/dev/xmega_tty")
sliced_path = linked_path[9:] # returns pts number the tty port is linked to

#convert_one = '/dev/pts/' + str(port_one)
convert_two = '/dev/pts/' + str(sliced_path)

#print 'Streaming from ', convert_one
print 'Waiting for signal from xmega driver at ' , convert_two

#read_ser = serial.Serial(convert_one, 230400, timeout=1)
read_ser_n = serial.Serial(convert_two)

first_types = ['0x5e','0xC0','0x43','0x80']
second_types = ['0xC0','0xC0']
third_types = ['0x01','0x02','0x03','0x04']

Messgage = []

while 1:

	raw_input = read_ser_n.read()
	convert_to_hex = hex(ord((raw_input))) # Need to set values to fill Message array and iterate through it

# ------------------------------------------------------------------------------

	for x in range(0, len(first_types)):
		if convert_to_hex == first_types[x]:
			print convert_to_hex # -------------- Testing print statement
			boo = True
		else:
			boo = False

	if boo == False:
		print "Message Type Error"

# ------------------------------------------------------------------------------

	boo = True
	for x in range(0, len(second_types)):
		if convert_to_hex == second_types[x]:
			print convert_to_hex # -------------- Testing print statement
			boo = True
		else: 
			boo = False

	if boo == False:
		print "Second Type Error"

# ------------------------------------------------------------------------------

	boo = True
	for x in range(0, len(third_types)):
		if convert_to_hex == third_types[x]:
			print convert_to_hex # -------------- Testing print statement
			boo = True
		else:
			 boo = False

	if boo == False:
		print "Third Type Error"

# ------------------------------------------------------------------------------


		
	


