import subprocess
import sys
import serial
import time
import os

# must install socat to operate
# www.dest-unreach.org/socat/
# or 
# sudo apt-get install socat

print 
print "Xmega simulation setup..."
print 
print "The proper PTS ports are now displayed in the popup terminal"
print "Locate the port numbers and enter them now"


# subprocess to open fake TTy Ports
subprocess.call(["chmod", "u+x", "./com_ports_on.sh"])

subprocess.Popen(["xterm", "-e", "./com_ports_on.sh"])

time.sleep(.5)

subprocess.call(["chmod", "666", "/dev/xmega_tty"])

linked_path = os.path.realpath("/dev/xmega_tty")

sliced_path = linked_path[9:] # returns pts number the tty port is linked to

#convert_one = '/dev/pts/' + str(port_one)
convert_two = '/dev/pts/' + str(sliced_path)

#print 'Streaming from ', convert_one
print 'Waiting for signal from xmega driver at ' , convert_two

#read_ser = serial.Serial(convert_one, 230400, timeout=1)
read_ser_n = serial.Serial(convert_two)

while 1:

	x = read_ser_n.read()
	he = hex(ord((x)))
	print he       
