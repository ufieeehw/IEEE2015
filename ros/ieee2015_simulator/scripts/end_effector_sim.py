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
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from dynamixel_msgs.msg import JointState

to_radians_one = 512
to_radians_two = 512

side_control = 1
large_control = 1
small_control = 1

past_location_one = 0
past_location_two = 0


SCREEN_DIM = (750, 750)
ORIGIN = np.array([SCREEN_DIM[0]/2.0, SCREEN_DIM[1]/2.0])


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

    temp = int(param * 3.405)

    if temp < 1023:
        if servo == 1:
            past_location_one = temp
        if servo == 2:
            past_location_two = temp
        return temp
    if temp > 1024:
        if servo == 1:
            return past_location_one
        if servo == 2:
            return past_location_two


class END(object):

    def __init__(self):

        rospy.init_node('SCARA_simulator')
        self.base = np.array([0.0, 0.0], np.float32)
        self.point = np.array([0.0, 0.0], np.float32)

        self.point_two = np.array([0.0, 0.0], np.float32) 

        self.starting = np.array([2, -3.5], np.float32)
        self.starting_two = np.array([-2, -3.5], np.float32)
        self.starting_three = np.array([0, 1], np.float32)

        self.desired_pos = rospy.Subscriber('/end_des_pose', PointStamped, self.got_des_pose)
        self.desired_pos_two = rospy.Subscriber('/end_des_pose_two', PointStamped, self.got_des_pose_two)


    def got_des_pose(self, msg):
        '''Recieved desired arm pose'''
        self.point = (msg.point.x, msg.point.y)
        global to_radians_one

        to_radians_one = math.atan2(msg.point.y, msg.point.x)

        print to_radians_one

        degrees_one = to_degrees(to_radians_one)
        xl_format = check_size(degrees_one, 1)

        to_radians_one = xl_format

        #print "TARGETING POSITION:   ({}, {})".format(*self.point) 
        #print "LARGE SERVO POSITION ", degrees_one, "radians"
        print "LARGE SERVO POSITION: ", xl_format

        base_pub = rospy.Publisher('/ieee2015_end_effector_servos', Num, queue_size=1)
        base_pub.publish(side_control, to_radians_one, to_radians_two, large_control, small_control)

    def got_des_pose_two(self, msg):
        '''Recieved desired arm pose'''
        self.point = (msg.point.x, msg.point.y)
        global to_radians_two

        to_radians_two = math.atan2(msg.point.y, msg.point.x)  * (180/np.pi) + 60

        degrees_two =to_degrees(to_radians_two)
        xl_format = check_size(degrees_two, 2)

        to_radians_two = xl_format

        #print "TARGETING POSITION:   ({}, {})".format(*self.point) 
        #print "SMALL SERVO moved to ", degrees_two, "radians"
        print "SMALL SERVO POSITION: ", xl_format

        base_pub = rospy.Publisher('/ieee2015_end_effector_servos', Num, queue_size=1)
        base_pub.publish(side_control, to_radians_one, to_radians_two, large_control, small_control)

    def draw(self, display, new_base=(0, 0)):
        '''Draw the whole arm'''
        # Update positions given current 

        pygame.draw.circle(display, (255, 255, 50), round_point(self.base), int(300), 2)
        pygame.draw.line(display, (255, 162, 0), round_point(self.base), round_point(self.point), 3)
        pygame.draw.line(display, (255, 130, 0), round_point(self.base), round_point(self.point_two), 3)
        pygame.draw.line(display, (255, 255, 255), round_point(self.base), round_point(self.starting), 1)
        pygame.draw.line(display, (255, 255, 255), round_point(self.base), round_point(self.starting_two), 1)
        pygame.draw.line(display, (255, 255, 255), round_point(self.base), round_point(self.starting_three), 1)



def main():
    '''In principle, we can support an arbitrary number of servos in simulation'''
    end_one = [END()]

    global side_control
    global large_control
    global small_control

    display = pygame.display.set_mode(SCREEN_DIM)

    des_pose_pub_end = rospy.Publisher('/end_des_pose', PointStamped, queue_size=1)
    des_pose_pub_end_two = rospy.Publisher('/end_des_pose_two', PointStamped, queue_size=1)



    def publish_des_pos_end(pos):
        '''Publish desired position of the arm end-effector based on click position'''
        des_pose_pub_end.publish(
            PointStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/robot',
                ),
                point=Point(
                    x=pos[0], 
                    y=pos[1], 
                    z=0
                )
            )
        )

    def publish_des_pos_two(pos):
        des_pose_pub_end_two.publish(
            PointStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/robot',
                ),
                point=Point(
                    x=pos[0], 
                    y=pos[1], 
                    z=0
                )
            )
        )

    clock = pygame.time.Clock()

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    pt = pygame.mouse.get_pos()
                    publish_des_pos_end(unround_point(pt))
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    pt = pygame.mouse.get_pos()
                    publish_des_pos_two(unround_point(pt))
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_z:
                    side_control = 1
                    print "CONTROL MODE: Wheel"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_x:
                    side_control = 2
                    print "CONTROL MODE: Angle"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    large_control = 1
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    large_control = 2
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_e:
                    small_control = 1
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    small_control = 2

        t = time.time()

        for arm in end_one:
            arm.draw(display)

        pygame.display.update()
        clock.tick(20)
        display.fill((0, 0, 0))


if __name__ == '__main__':
    main()