#!/usr/bin/python
'''
The purpose of this script is to draw the angle, desired velocity and current velocity vectors, and write position and angle in text.
See the visualizer in ieee2015_simulator/scripts for vehicle visualization, 
 motion simulation, and everything else that matters to all of the other tasks, 

'''
from __future__ import division
## Math
import numpy as np
import math
## Rendering
import pygame
import time
## Ros
from tf import transformations as tf_trans
import rospy
## Ros msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3


SCREEN_DIM = (250, 250)
SCALE = 1

def round_point((x, y)):
    return int(SCALE*x+SCREEN_DIM[0]+.5), int(SCALE*-y+SCREEN_DIM[1]+.5)


class Line(object):
    def __init__(self, point1, point2, color=(255,255,255)):
        self.point1 = point1
        self.point2 = point2

    def draw(self, display):
        '''draw object method'''
        pygame.draw.line(display, self.color, round_point(self.point1), round_point(self.point2))


class Text_Box(object):
    def __init__(self, pos=(0,0), color=(255,255,255)):
        pygame.font.init()
        self.font = pygame.font.SysFont("monospace", 15)

        # Render text
        # label = myfont.render("Some text!", 1, (255,255,0))
        # screen.blit(label, (100, 100))

    def draw(self, display, text):
        '''draw object method'''
        lines = self.text.splitlines()
        width = height = 0
        for l in lines:
            width = max(width, font.size(l)[0])
            height += font.get_linesize()

        #create 8bit image for non-aa text..
        # img = pygame.Surface((width, height), 0, 8)
        # img.set_palette([bg, color])

        # Render each line
        height = 0
        for l in lines:
            t = font.render(l, 0, color, bg)
            display.blit(t, (0, height))
            height += font.get_linesize()



class Visualizer(object):
    def __init__(self):
        rospy.init_node('controller_visualization')

        twist_topic = 'desired_velocity'
        self.twist_sub = rospy.Subscriber(twist_topic, Twist, self.got_twist) 
        
        # Current pose sub
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.got_pose)
        self.desired_pose_sub = rospy.Subscriber('desired_pose', Pose, self.got_desired_pose)

        # Nonealizations
        self.ang_vel = self.lin_vel = self.des_lin_vel = self.des_ang_vel = None

        self.objs = []

    def add_to_list(f):
        '''Half this shit I don't know what I was doing...'''
        self.objs.append(f)

    def got_twist(self, msg):
        self.lin_vel = np.array([msg.linear.x, msg.linear.y])
        self.ang_vel = msg.angular.z

    def got_pose(self, msg):
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]

    def got_desired_pose(self, msg):
        self.des_position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.des_yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]

    @self.add_to_list
    def draw_velocities(self, display):
        # Twist
        x,y = self.lin_vel
        des_vel  = Line(x, y, (250, 50, 100))
        des_vel.draw(display)

        # x2, y2 = self.lin_vel
        # des_ang_vel = Line(x2, y2, (250, 50, 250))

    @self.add_to_list
    def draw_rotation(self, display):
        length = 100
        x,y = (np.cos(self.yaw) * length, np.sin(self.yaw) * length)
        rot_line = Line(x, y, (250, 250, 50))
        rot_line.draw(display)

        length_2 = 150
        x2,y2 = (np.cos(self.des_yaw) * length_2, np.sin(self.des_yaw) * length_2)
        des_rot_line = Line(x, y, (100, 250, 200))
        des_rot_line.draw(display)

    @self.add_to_list
    def draw_twist(self, display):
        pass

    @self.add_to_list
    def draw_text(self, display):
        pass


    @self.add_to_list
    def draw_bot(self, display):
        pass

    def render(self, display):
        for draw_func in self.draw_funcs:
            draw(display)


def main():
    # Initialize pygame
    clock = pygame.time.Clock()
    display = pygame.display.set_mode(SCREEN_DIM)
    
    visualizer = Visualizer()
    clock = pygame.time.Clock()
    while not rospy.is_shutdown():
        dynamixels.publish_servo_positions()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
        
        t = time.time()
        visualizer.render(display)
                
        pygame.display.update()
        clock.tick(60)
        display.fill((0, 0, 0))

main()