import subprocess
import sys
import serial

# must install socat to operate
# www.dest-unreach.org/socat/
# or 
# sudo apt-get install socat


# subprocess to open fake TTy Portschomd 
subprocess.call(["chmod", "u+x", "./com_ports_on.sh"])

subprocess.Popen(["xterm","-hold", "-e", "./com_ports_on.sh"])

#subprocess.call(["grome-terminal","-x", "sudo", "chmod", "666", "/dev/xmeega_tty"])





print 
print "Xmega simulation setup..."
print 
print "The proper PTS ports are now displayed in the popup terminal"
print "Locate the port numbers and enter them now"

#port_one = input('PTY Port 1: ') 
port_two = input('PTY Port 2: ')

#convert_one = '/dev/pts/' + str(port_one)
convert_two = '/dev/pts/' + str(port_two)

#print 'Streaming from ', convert_one
print 'Streaming from' , convert_two

#read_ser = serial.Serial(convert_one, 230400, timeout=1)
read_ser_n = serial.Serial(convert_two)

while 1:

	x = read_ser_n.read()
	he = hex(ord((x)))
	print he       
