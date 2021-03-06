#!/usr/bin/python
## Math
import numpy as np
## Display
import pygame
import time
import math
## Ros
import rospy
from tf import transformations as tf_trans
## Ros Msgs
from std_msgs.msg import Header, Float64, Int64
from ieee2015_end_effector_servos.msg import Num
from ieee2015_end_effector_servos.msg import high_level
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from dynamixel_msgs.msg import JointState

to_radians_one = 512
to_radians_two = 512
to_radians_three = 512

side_control = 1
large_control = 1
small_control = 1

past_location_one = 0
past_location_two = 0
past_location_three = 0




def round_point((x, y)):
    '''Round and change point to centered coordinate system'''
    return map(int, ((1000 * x) + ORIGIN[0], -(1000 * y) + ORIGIN[1]))

def unround_point((x, y)):
    '''Change center-origin coordinates to pygame coordinates'''
    return ((x - ORIGIN[0])/1000.0, (-y + ORIGIN[1])/1000.0)

def to_degrees(param):
        if param < 0:
            temp = 180 - math.fabs(param)
            temp2 = temp * 2
            return math.fabs(param) + temp2
        else:
            return param

def check_size(param, servo):

    global past_location_one
    global past_location_two
    global past_location_three

    temp = int(param * 3.405)

    if temp < 1023:
        if servo == 1:
            past_location_one = temp
        if servo == 2:
            past_location_two = temp
        if servo == 3:
            past_location_three = temp
        return temp
    if temp > 1024:
        if servo == 1:
            return past_location_one
        if servo == 2:
            return past_location_two
        if servo == 3:
            return past_location_three

def get_some(msg):

    global to_radians_one
    global to_radians_two
    global to_radians_three

    large_conv = msg.large_radians * (180/np.pi) + 60
    small_conv = msg.small_radians * (180/np.pi) + 60
    wrist_conv = msg.wrist_radians * (180/np.pi) + 60

    degrees_one = to_degrees(large_conv)
    degrees_two = to_degrees(small_conv)
    xl_format_one = check_size(degrees_one, 1)
    xl_format_two = check_size(degrees_two, 2)

    to_radians_one = xl_format_one
    to_radians_two = xl_format_two

    master_sub = rospy.Publisher('ieee2015_end_effector_servos', Num, queue_size=1)
    master_sub.publish(to_radians_one, to_radians_two, to_radians_three)

def get_some_two(msg):

    global to_radians_one
    global to_radians_two
    global to_radians_three

    wrist_conv = msg.current_pos * (180/np.pi) + 60

    degree_three = to_degrees(wrist_conv)
    xl_format_three = check_size(degree_three, 3)

    to_radians_three = xl_format_three

    master_sub = rospy.Publisher('ieee2015_end_effector_servos', Num, queue_size=1)
    master_sub.publish(to_radians_one, to_radians_two, to_radians_three)


def main():

    rospy.init_node('listener', anonymous=True)
    master_sub = rospy.Subscriber('/end_efffector_angles_sub', high_level, get_some)
    master_base_sub = rospy.Subscriber('/robot/base_controller/state', JointState, get_some_two)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()