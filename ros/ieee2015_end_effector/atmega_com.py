import subprocess
import sys
import serial
import time
import codecs


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

    Message = serial.read(1)
    
    for x in range(0, byte_num):

        temp = hex_determine(int.from_bytes(Message, byteorder='little'))
        print (temp)
        read_from_serial(byte_num - 1)


# <--------------------------------------------------------------------------------------->


def available_ports():
    subprocess.call(["python", "-m", "serial.tools.list_ports"])


# <--------------------------------------------------------------------------------------->


def solenoid_message():

    serial.write(str('j').encode())
    time.sleep(.04)
    serial.write(str('l').encode())

def servo_message():

    serial.write(str('u').encode())
    time.sleep(.04)
    serial.write(str('i').encode())


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

serial = serial.Serial(sys.argv[1], baudrate=9600, timeout=0)

# <-------------------------------------MAIN LOOP----------------------------------------->

while 1:

    solenoid_message()
    read_from_serial(2)
