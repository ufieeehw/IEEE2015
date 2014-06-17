#!/usr/bin/python
## Math
import numpy as np
import math
## Display
import pygame
import time
## Ros
import rospy
from tf import transformations as tf_trans
## Ros Msgs
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion

SCREEN_DIM = (500,500)
ORIGIN = np.array([SCREEN_DIM[0]/2.0, SCREEN_DIM[1]/2.0])

def round_point((x,y)):
    return map(int, (x, -y) + ORIGIN)

def norm_angle_diff(ang_1, ang_2):
    '''norm_angle_diff(ang_1, ang_2)
    -> Normalized angle difference, constrained to [-pi, pi]
    '''
    return (ang_1 - ang_2 + math.pi) % (2 * math.pi) - math.pi


class Line(object):
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2
    
    def update(self, point1, point2):
        self.point1, self.point2 = point1, point2
    
    def draw(self, display):
        pygame.draw.line(display, (255, 255, 255), round_point(self.point1), round_point(self.point2))
    
    @property
    def end(self):
        '''Farthest point from origin'''
        return max(self.points, key=lambda v: np.linalg.norm(v - (0,0)))
    @property
    def start(self):
        '''Closest point to origin'''
        return min(self.points, key=lambda v: np.linalg.norm(v - (0,0)))
    @property
    def points(self):
        return np.array([self.point1, self.point2], np.float32)

    def __getitem__(self, key):
        return self.points[key]

    @classmethod
    def dotproduct(v1, v2):
      return sum((a*b) for a, b in zip(v1, v2))
    @classmethod
    def length(v):
      return math.sqrt(dotproduct(v, v))
    @classmethod
    def angle(line1, line2):
      return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))


class SCARA(object):
    def __init__(self):
        rospy.init_node('SCARA_simulator')
        self.base = numpy.array([0.0, 0.0])

        length1, length2 = 30, 20
        self.joint1 = Line(np.array([0, 0]), (length1, 0))
        self.joint2 = Line(self.joint1.end, self.joint1.end + (length2, 0))

        self.angle1, self.angle2 = 0.0,0.0

    def update(self):
        self.new_end_1 = (np.tan(self.angle1))

        self.joint1.update(self.base, self.new_end_1)
        self.joint2.update(self.joint1.end, new_end_2)

    def draw(self, display):
        self.joint1.draw(display)
        self.joint2.draw(display)


def main():
    arm1 = SCARA()
    arms = [arm1]

    print 'Joint 1 start', arm1.joint1.start
    print 'Joint 1 end', arm1.joint1.end
    print 'Joint 2 start', arm1.joint2.start
    print 'Joint 2 end', arm1.joint2.end

    display = pygame.display.set_mode(SCREEN_DIM)
    
    clock = pygame.time.Clock()
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
        


        t = time.time()
        for arm in arms:
            arm.draw(display)
        
        display.fill((0, 0, 0))
        pygame.display.update()
        clock.tick(60)

if __name__ == '__main__':
    main()