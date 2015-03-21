#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
import subprocess
import sys
import serial
import time
import codecs


serial_port = 0
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
    check = abs(ord(param1) - ord(param2))
    return check


# <--------------------------------------------------------------------------------------->


# <----------------------------- End of Functions Section 



# <-------------------------------------MAIN LOOP----------------------------------------->

def callback(data):

    global serial_port

    if data.data == 1:       
        check = checksum('j', 'k')
        serial_port.write(str('j').encode())
        serial_port.write(str('k').encode())
        serial_port.write(str(check).encode())
    if data.data == 0:
        check = checksum('j', 'l')
        serial_port.write(str('j').encode())
        serial_port.write(str('l').encode())
        serial_port.write(str(check).encode())

def listener():

    global serial_port

    serial_port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=0)

    rospy.init_node('listener', anonymous=True)


    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.

    rospy.Subscriber("end_effector_solenoids", Int64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

