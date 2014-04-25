#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
import pygame
from std_msgs.msg import Float64

width = 250
height = 1300/8
bgcolor = 192, 192, 192
linecolor = 255, 0 , 0

screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

def render(dr):
    last_position = (0,0)
    while not rospy.is_shutdown():
       end_position =  (last_position[0]+dr[0], last_position[1]+dr[1])
       pygame.draw.line(screen, linecolor, last_position, end_position, 1)
       last_position = end_position; 

       pygame.display.flip()
       clock.tick(240)

def callback(data):
    render(data.data)

    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('etch_visualizer', anonymous=True)

    rospy.Subscriber("random_movement", Float64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
