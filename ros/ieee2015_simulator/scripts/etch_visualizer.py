#!/usr/bin/env python

import rospy
import pygame
from std_msgs.msg import Float64
from ieee2015_simulator.msg import Float_List

#constants
SCREEN_WIDTH = 250
SCREEN_HEIGHT = 1300/8
BG_COLOR = 192, 192, 192
LINE_COLOR = 255, 0 , 0


class Renderer:
    '''
    Renderer class
    
    Creator:
        Aaron Marquez - UF CISE (2016)
        (Special thanks to Rev. Jacob Panikulam)
    Function:
        Draws on awesome simulated etch-a-sketch

    '''
    def __init__(self):
        self.last_position = [SCREEN_WIDTH/2,SCREEN_HEIGHT/2]
        
        rospy.init_node('etch_visualizer', anonymous=True)
        rospy.Subscriber("random_movement", Float_List, self.callback)
  
    def render(self,dr):
        end_position = (self.last_position[0]+dr[0], self.last_position[1]+dr[1])
        
        pygame.draw.line(screen, LINE_COLOR, self.last_position, end_position, 1)
        self.last_position = end_position; 
        pygame.display.flip()
        #max frames per second
        clock.tick(240)
 
    def callback(self, subscription):
        dr = (subscription.float_list[0].data, subscription.float_list[1].data)
        self.render(dr)
    
if __name__ == '__main__':
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()
    my_renderer = Renderer()
    rospy.spin()
