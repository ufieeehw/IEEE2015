import subprocess
import sys
import serial
import time
import codecs

#defines 


# <--------------------------------Function Definitions--------------------------------->

# function to convert character into hex value

def hex_determine(message):

    if type(message) is str:
        convert = hex(ord((message)))
        return convert
    if type(message) is int:
        convert = hex(message)
        return convert

# <--------------------------------------------------------------------------------------->


def read_from_serial(byte_num):

    Message = serial_port.read(byte_num)

    temp = hex_determine(int.from_bytes(Message, byteorder='little'))
    print (temp)


# <--------------------------------------------------------------------------------------->


def available_ports():
    subprocess.call(["python", "-m", "serial.tools.list_ports"])

# <--------------------------------------------------------------------------------------->

def checksum(param1, param2):
	#take two parameter
	#subaract one from the other and take abs value
	check = abs(ord(param1) -ord(param2))
	return check

def solenoid_message(in_or_out):

    if in_or_out == 1:
        check = checksum('j', 'k')
        serial_port.write(str('j').encode())
        serial_port.write(str('k').encode())
       	serial_port.write(str(check).encode())


    if in_or_out == 0:
        check = checksum('j', 'l')
        serial_port.write(str('j').encode())
        serial_port.write(str('l').encode())
       	serial_port.write(str(check).encode())

    read_from_serial(2)	

def servo_message():

    serial_port.write(str('u').encode())
    time.sleep(.04)
    serial_port.write(str('i').encode())


# <--------------------------------------------------------------------------------------->


# <----------------------------- End of Functions Section 

if len(sys.argv) != 2:
    print ("Please supply an argument")
    print ("Use -p to list available ports")
    print ("Supply desired COM port as argument")
    sys.exit(0)

if sys.argv[1] == "-p":
    available_ports()
    sys.exit(0)

serial_port = serial.Serial(sys.argv[1], baudrate=9600, timeout=0)

# <-------------------------------------MAIN LOOP----------------------------------------->

while 1:

    solenoid_message(1)
    solenoid_message(1)
    solenoid_message(1)
    solenoid_message(1)
    solenoid_message(1)
    time.sleep(1)
    solenoid_message(0)
    solenoid_message(0)
    solenoid_message(0)
    solenoid_message(0)
    solenoid_message(0)
    time.sleep(1)
