#!/usr/bin/python
from __future__ import division
## Math
import numpy as np
import math

## Display
import pygame
import time

## Ros
import rospy
import os, sys

from geometry_msgs.msg import Twist, Pose, PoseStamped, Point, Quaternion
from tf import transformations as tf_trans
from std_msgs.msg import Header
from ieee2015_msgs.msg import Mecanum
from xmega_connector.srv import *

#constants
SCREEN_DIM = (700, 350)
fpath = os.path.dirname(os.path.realpath(__file__))
background = pygame.image.load(os.path.join(fpath, "stage.jpg"))
background = pygame.transform.scale(background, SCREEN_DIM)
rect = background.get_rect()

PXL_PER_METER = 50
#3779.527559055 #Change the number to the correct value!

"""Dimensions of the IEEE Competition
Course:
4 ft. x 8 ft.
1.2192 m x 1.8288 m

Robot:
1 ft. x 1 ft.
0.3048 m x 0.3048 m"""


dt = .5
radius = 10

ORIGIN = np.array([SCREEN_DIM[0]/2.0, SCREEN_DIM[1]/2.0])


## Function to calculate the cross-torque of an array of velocities (top left, top right, bottom left, bottom right)
def crosstorque(velocity):
    forcex = 0.0
    forcey = 0.0
    forcex += velocity[0] * np.sin(np.pi / 4)
    forcex += velocity[1] * np.sin(np.pi / 4)
    forcex += velocity[2] * np.sin(np.pi / 4)
    forcex += velocity[3] * np.sin(np.pi / 4)
    # N = kg m/s^2 = m/s * (r)
    # M_i = 
    forcey += velocity[0] * np.sin(np.pi / 4)
    forcey -= velocity[1] * np.sin(np.pi / 4)
    forcey += velocity[2] * np.sin(np.pi / 4)
    forcey -= velocity[3] * np.sin(np.pi / 4)



    return np.array([forcex, forcey])


## Class to define a robot object that moves with velocity, acceleration, and force
class Robot(object):

    def __init__(self, (x, y), height, width):
        self.position = np.array([x, y], dtype=np.float32)
        self.position += [0, (150 - height)]
        self.velocity = np.array([0, 0], dtype=np.float32)
        self.acceleration = np.array([0, 0], dtype=np.float32)
        self.force = np.array([0, 0], dtype=np.float32)

        # Implementation for rotation of object
        self.angle = 0
        self.omega = 0
        self.alpha = 0

        self.mecanum = np.array([0, 0, 0, 0], dtype=np.float32)

        rospy.init_node('simulation', anonymous=True)
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        

        self.height = height
        self.width = width

        a = +(self.width / 2)
        b = -(self.width / 2)
        c = +(self.height / 2)
        d = -(self.height / 2)
        
        self.pointlist = map(lambda vector: np.array(vector, dtype=np.float32), [[b, d], [a, d], [a, c], [b, c]])
        
        rospy.Service('/xmega_connector/set_wheel_speeds', SetWheelSpeeds, self.set_wheel_speed_service)


    def set_wheel_speed_service(self, ws_req):
        
        if abs(ws_req.wheel1) > .00000000001 and abs(ws_req.wheel2) > .00000000000001 and abs(ws_req.wheel3) > .00000000000001 and abs(ws_req.wheel4) > .00000000000001:
            self.mecanum[0] = ws_req.wheel1
            self.mecanum[1] = ws_req.wheel2 
            self.mecanum[2] = ws_req.wheel3 
            self.mecanum[3] = ws_req.wheel4

        else:
            self.mecanum[0] = 0
            self.mecanum[1] = 0
            self.mecanum[2] = 0
            self.mecanum[3] = 0



        #print('Wheel speeds set')
        return SetWheelSpeedsResponse()

    def update(self):
        self.publish_pose()

        # Update velocity and position
        #self.position[0] += self.velocity[0] * dt
        #self.position[1] += self.velocity[1] * dt
        #self.velocity += self.acceleration * dt

        self.position += self.force * dt
        #self.acceleration = self.force

        # Update rotation of object
        self.angle += self.omega * dt
        self.omega += self.alpha * dt
        self.force = crosstorque(self.mecanum)  
        

        # Makes sure the object stays in the window
        if self.position[0] + (self.width / 2) >= 700 or self.position[0] - (self.width / 2) <= 0:
            self.velocity[0] *= -1
        if self.position[1] + (self.height / 2) >= 350 or self.position[1] - (self.height / 2) <= 0:
            self.velocity[1] *= -1
    

    def rotate(self):
        # Rotates the object by some angle, in degrees, clockwise
        radangle = math.radians(self.angle)
        rotmatrix = np.matrix([[math.cos(radangle), -math.sin(radangle)], [math .sin(radangle), math.cos(radangle)]])
        templist = []

        """one = np.matrix([
                        [1, 0, 50], 
                        [0, 1, 0], 
                        [0, 0, 1]])
        two = np.matrix([[math.cos(radangle), -math.sin(radangle), 0], [math.sin(radangle), math.cos(radangle), 0], [0, 0, 1]])
        three = np.matrix([
                        [1, 0, -50], 
                        [0, 1, 0], 
                        [0, 0, 1]])

        rotmatrix = one * two * three"""

        for point in self.pointlist:
            matpoint = np.matrix(point).T
            matpoint = rotmatrix * matpoint

            point = np.array(matpoint.T)[0]
            templist.append(point)

        self.pointlist = templist

    def publish_pose(self):
        '''Publish Pose'''
        _orientation = tf_trans.quaternion_from_euler(0, 0, self.angle)
        self.pose_pub.publish(
            PoseStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/world',
                ),
                pose = Pose(
                    position = Point(self.position[0], self.position[1], 0.0),
                    orientation = Quaternion(*_orientation) #Radians
                )
            )
        )

    def draw(self, display):
        # polygon(Surface, color, pointlist, width=0) -> Rect
        # pointlist = [(1, 2), (7, 9), (21, 50)]
        """roundedlist = []

        for point in self.pointlist:
            roundedlist.append(round_point(point + self.position))

        print roundedlist"""

        """Change position coordinates from meters to pixels."""

        listInMeters = (self.position + self.pointlist)

        display.blit(background, (0,0))

        pygame.draw.polygon(display, (0,255,0), listInMeters, 0)


class World:
    def __init__(self, Robot, waypoints):
        self.Robot = Robot
        self.waypoints = waypoints

def main():
    pygame.init()
    display = pygame.display.set_mode(SCREEN_DIM)
    background.convert()
    dimensions = (background.get_width(), background.get_height())

    clock = pygame.time.Clock()
    Rob = Robot((50, 50), 50, 50) 

    
    
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.MOUSEBUTTONDOWN:

                Rob.desired_pose_pub = rospy.Publisher('desired_pose', PoseStamped, queue_size=10)

                pt = pygame.mouse.get_pos()
                # Publish this coordinate in meters as the desired pose
                print 'publishing desired pose'
                # _orientation = tf_trans.quaternion_from_euler(0, 0, Rob.angle)
                _orientation = tf_trans.quaternion_from_euler(0.0, 0.0, 0.0)
                Rob.desired_pose_pub.publish(
                    PoseStamped(
                        header = Header(
                            stamp=rospy.Time.now(),
                            frame_id='/world'
                        ),
                        pose = Pose(
                            # Update the position to reflect meters per second, not pixels
                            position = Point(pt[0], pt[1], 0.0),
                            orientation = Quaternion(*_orientation) #Radians
                        )
                    )
                )

        Rob.rotate()
        Rob.draw(display) 
        Rob.update()


        pygame.display.update()
        clock.tick(20)
        display.fill((0, 0, 0))


if __name__ == '__main__':
    main()