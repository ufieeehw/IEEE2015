#!/usr/bin/env python
from __future__ import division
import rospy
import pygame
from std_msgs.msg import Float64
import numpy as np

#constants
SCREEN_WIDTH = 250
SCREEN_HEIGHT = 1300//8
BG_COLOR = 192, 192, 192
LINE_COLOR = 255, 0 , 0


class Renderer(object):
    '''
    Renderer class
    
    Creator:
        Aaron Marquez - UF CISE (2016)
        (Special thanks to Rev. Jacob Panikulam)
    Function:
        Draws on awesome simulated etch-a-sketch

    '''
    def __init__(self):
        self.last_position = [0.0, 0.0]
        rospy.init_node('etch_visualizer', anonymous=True)
        rospy.Subscriber('end_effector/left/angle', Float64, self.left, queue_size=1)
        rospy.Subscriber('end_effector/right/angle', Float64, self.right, queue_size=1)
        self.dx, self.dy = 0.0, 0.0

    def render(self, screen):
        end_position = (self.last_position[0] + int(self.dx), self.last_position[1] + int(self.dy))
        pygame.draw.line(screen, LINE_COLOR, self.last_position, end_position, 1)
        self.last_position = end_position; 
        self.dx, self.dy = 0.0, 0.0
 
    def left(self, msg):
        self.dx = (msg.data / (3 * np.pi / 2)) * 200

    def right(self, msg):
        self.dy = (msg.data / (3 * np.pi / 2)) * 200
    
if __name__ == '__main__':
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()
    my_renderer = Renderer()

    left = rospy.Publisher('end_effector/left/angle', Float64)
    right = rospy.Publisher('end_effector/right/angle', Float64)


    pt = np.array([0.0, 0.0])
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()

            if event.type == pygame.KEYDOWN:
                if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                    exit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                mpos = pygame.mouse.get_pos()
                delta = ((mpos - pt) / 200) * (3 * np.pi / 2)
                left.publish(delta[0])
                right.publish(delta[1])
                pt = np.array(mpos)



        my_renderer.render(screen)
        pygame.display.flip()
        # max frames per second
        clock.tick(240)

