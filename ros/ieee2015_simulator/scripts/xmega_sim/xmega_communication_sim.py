import subprocess
import sys
import serial

# must install socat to operate
# www.dest-unreach.org/socat/
# or 
# sudo apt-get install socat


# subprocess to open fake COM Ports
subprocess.call(["chmod", "u+x", "./com_ports_on.sh"])
subprocess.Popen(["xterm","-hold", "-e", "./com_ports_on.sh"])

print 
print "Xmega simulation setup..."
print 
print "The proper PTS ports are now displayed in the popup terminal"
print "Locate the port numbers and enter them now"

port_one = input('PTY Port 1: ') 
print 'Port one is ', port_one
port_two = input('PTY Port 2: ')
print 'Port two is ', port_two
print port_one

convert_one = '/dev/pts/' + str(port_one)
convert_two = '/dev/pts/' + str(port_two)

#serial_1 = serial.Serial(port_one)
print convert_one
print convert_two

send_ser = serial.Serial(convert_two)

read_ser = serial.Serial(convert_two, 19200, timeout=1)

while 1:

	send_ser.write("streaming...")
	x = read_ser.read(14)
	print x