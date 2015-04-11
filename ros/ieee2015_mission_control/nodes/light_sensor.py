#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
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
    return temp


def listener():

    global serial_port

    serial_port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=0)

    rospy.init_node('listener', anonymous=True)


    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.

    pup = rospy.Publisher("light_start", bool, queue_size = 1)


    temp = read_from_serial(1)
    while(temp != 0xFF):
        temp = read_from_serial(1)
        pup.publish(False);

    pup.publish(True);

if __name__ == '__main__':
    listener()
