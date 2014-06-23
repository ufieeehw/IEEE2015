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

def print_in(f):
    print("Defining " + f.func_name)
    def print_on_entry(*args, **kwargs):
        print("Executing " + f.func_name + " with arguments " + str(args) + " and kwargs " + str(kwargs))
        result = f(*args, **kwargs)
        print("Returning " + str(result))
        return(result)
    return(print_on_entry)

def round_point((x,y)):
    return map(int, (x + ORIGIN[0], -y + ORIGIN[1]))

def norm_angle_diff(ang_1, ang_2):
    '''norm_angle_diff(ang_1, ang_2)
    -> Normalized angle difference, constrained to [-pi, pi]
    '''
    return (ang_1 - ang_2 + math.pi) % (2 * math.pi) - math.pi


class Line(object):
    '''->Make a *joint* class that supports multiple children, parents, and local reference frames
        Can use the samething to back-out the end pose from angles, etc
    '''
    def __init__(self, point1, point2, color=(200,200,200)):
        self.point1 = np.array(point1, np.float32)
        self.point2 = np.array(point2, np.float32)
        print "Initializing", self.point1, self.point2
        self.color = color
    
    def update(self, point1, point2):
        self.point1 = np.array(point1, np.float32)
        self.point2 = np.array(point2, np.float32)
    
    def draw(self, display):
        pygame.draw.line(display, self.color, round_point(self.point1), round_point(self.point2), 4)
    
    @property
    def end(self):
        '''Farthest point from origin'''
        return np.array(max(self.points, key=lambda v: np.linalg.norm(np.array(v) - (0,0))))
    @property
    def start(self):
        '''Closest point to origin'''
        return np.array(min(self.points, key=lambda v: np.linalg.norm(np.array(v) - (0,0))))
    @property
    def points(self):
        return [self.point1, self.point2]
    @property
    def norm(self):
        return np.linalg.norm(np.array(self.point2)-np.array(self.point1))

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
        self.base = np.array([0.0, 0.0])

        length1, length2 = 100, 100
        self.joint1 = Line(self.base, (length1, 0), color=(100,100,100))
        self.joint1.end
        self.joint2 = Line(self.joint1.end, self.joint1.end + (length2, 0), color=(200,200,100))

        self.elbow_sub = rospy.Subscriber('arm_elbow_angle', Float32, self.got_elbow_angle)
        self.base_sub = rospy.Subscriber('arm_base_angle', Float32, self.got_base_angle)

        self.error_sub = rospy.Subscriber('arm_des_pose', PoseStamped, self.got_des_pose)

        self.angle1, self.angle2 = 0.0 , 0.0
        self.des_pose = self.joint2[1]

    def got_des_pose(self, msg):
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
    @print_in
    def got_elbow_angle(self, msg):
        self.angle1 = msg.data
    @print_in
    def got_base_angle(self, msg):
        self.angle2 = msg.data

    def update(self, center=(0,0)):
        # TODO:
        # Make this non-instantaneous

        # Update elbow (end_1)
        self.base = center

        base_local_pos = self.joint1.norm * np.array([np.cos(self.angle1), np.sin(self.angle1)])
        self.new_end_1 = base_local_pos + self.base

        # Update endpoint as sum of base angle and elbow angle
        total_elbow_angle = self.angle1 + self.angle2

        # Superimpose positions
        elbow_local_pos = self.joint2.norm * np.array([np.cos(total_elbow_angle), np.sin(total_elbow_angle)])
        self.new_end_2 = self.new_end_1 + elbow_local_pos

        self.joint1.update(self.base, self.new_end_1)
        self.joint2.update(self.joint1.end, self.new_end_2)

    def draw(self, display, new_base=(0,0)):
        self.update(new_base)
        self.joint1.draw(display)
        self.joint2.draw(display)



def main():
    arm1 = SCARA()
    arms = [arm1]

    display = pygame.display.set_mode(SCREEN_DIM)

    clock = pygame.time.Clock()
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        t = time.time()
        for arm in arms:
            arm.draw(display)
        
        pygame.display.update()
        clock.tick(20)
        display.fill((0, 0, 0))


if __name__ == '__main__':
    main()