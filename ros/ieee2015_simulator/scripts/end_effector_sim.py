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
control_one = 2
control_two = 2

SCREEN_DIM = (750, 750)
ORIGIN = np.array([SCREEN_DIM[0]/2.0, SCREEN_DIM[1]/2.0])


def round_point((x, y)):
    '''Round and change point to centered coordinate system'''
    return map(int, ((1000 * x) + ORIGIN[0], -(1000 * y) + ORIGIN[1]))

def unround_point((x, y)):
    '''Change center-origin coordinates to pygame coordinates'''
    return ((x - ORIGIN[0])/1000.0, (-y + ORIGIN[1])/1000.0)


class END(object):

    def __init__(self):

        rospy.init_node('SCARA_simulator')
        self.base = np.array([0.0, 0.0], np.float32)
        self.point = np.array([0.0, 0.0], np.float32)

        self.point_two = np.array([0.0, 0.0], np.float32) 

        self.starting = np.array([2, 0], np.float32)
        self.starting_two = np.array([-2, 0], np.float32)
        self.starting_three = np.array([0, 1], np.float32)

        self.desired_pos = rospy.Subscriber('/end_des_pose', PointStamped, self.got_des_pose)
        self.desired_pos_two = rospy.Subscriber('/end_des_pose_two', PointStamped, self.got_des_pose_two)

    def got_des_pose(self, msg):
        '''Recieved desired arm pose'''
        self.point = (msg.point.x, msg.point.y)
        global to_radians_one

        to_radians_one = math.atan2(msg.point.y, msg.point.x)

        print "Targeting Base position: ({}, {})".format(*self.point) 
        print "LARGE SERVO moved to ", to_radians_one, "radians"

        to_radians_one = to_radians_one * 326

        base_pub = rospy.Publisher('/ieee2015_end_effector_servos', Num, queue_size=1)
        base_pub.publish(control_one, control_two, to_radians_one, to_radians_two)

    def got_des_pose_two(self, msg):
        '''Recieved desired arm pose'''
        self.point = (msg.point.x, msg.point.y)
        global to_radians_two

        to_radians_two = math.atan2(msg.point.y, msg.point.x)

        print "Targeting Base position: ({}, {})".format(*self.point) 
        print "SMALL SERVO moved to ", to_radians_two, "radians"

        to_radians_two = to_radians_two * 326

        base_pub = rospy.Publisher('/ieee2015_end_effector_servos', Num, queue_size=1)
        base_pub.publish(control_one, control_two, to_radians_one, to_radians_two)

    def draw(self, display, new_base=(0, 0)):
        '''Draw the whole arm'''
        # Update positions given current 

        pygame.draw.circle(display, (255, 255, 50), round_point(self.base), int(200), 2)
        pygame.draw.line(display, (255, 162, 0), round_point(self.base), round_point(self.point), 3)
        pygame.draw.line(display, (255, 130, 0), round_point(self.base), round_point(self.point_two), 3)
        pygame.draw.line(display, (255, 255, 255), round_point(self.base), round_point(self.starting), 1)
        pygame.draw.line(display, (255, 255, 255), round_point(self.base), round_point(self.starting_two), 1)
        pygame.draw.line(display, (255, 255, 255), round_point(self.base), round_point(self.starting_three), 1)


        

        # Draw the desired position circle




def main():
    '''In principle, we can support an arbitrary number of arms in simulation'''
    end_one = [END()]

    global control_one
    global control_two

    display = pygame.display.set_mode(SCREEN_DIM)

    des_pose_pub_end = rospy.Publisher('/end_des_pose', PointStamped, queue_size=1)
    des_pose_pub_end_two = rospy.Publisher('/end_des_pose_two', PointStamped, queue_size=1)

    solenoid_out = rospy.Publisher('/end_effector_solenoids', Int64, queue_size=1)



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
                if event.key == pygame.K_q:
                    solenoid_out.publish(1)
                    print "Solenoids OUT"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    solenoid_out.publish(0)
                    print "Solenoids IN"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_z:
                    control_one = 1
                    print "Control Mode SPIN"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_x:
                    control_two = 2
                    print "Control Mode ANGLE"

        t = time.time()

        for arm in end_one:
            arm.draw(display)

        pygame.display.update()
        clock.tick(20)
        display.fill((0, 0, 0))


if __name__ == '__main__':
    main()