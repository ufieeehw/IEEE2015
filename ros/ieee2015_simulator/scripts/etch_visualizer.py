#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
import pygame
from std_msgs.msg import Float64
from ieee2015_simulator.msg import Float_List
width = 250
height = 1300/8
bgcolor = 192, 192, 192
linecolor = 255, 0 , 0

screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

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
        self.last_position = [width/2,height/2]

        rospy.init_node('etch_visualizer', anonymous=True)
        rospy.Subscriber("random_movement", Float_List, self.callback)
  
    def render(self,dr):
        end_position =  (self.last_position[0]+dr[0], self.last_position[1]+dr[1])
        
        pygame.draw.line(screen, linecolor, self.last_position, end_position, 1)
        self.last_position = end_position; 

        pygame.display.flip()
        #max frames per second
        clock.tick(240)
 

    def callback(self, subscription):
        dr = (subscription.float_list[0].data, subscription.float_list[1].data)
        self.render(dr)

    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    # spin() simply keeps python from exiting until this node is stopped
    pass 
if __name__ == '__main__':
    #listener()
    my_renderer = Renderer()
    rospy.spin()
