#!/usr/bin/env python
# Software License Agreement (BSD License)
import rospy
import pygame
from geometry_msgs.msg import Twist

# 5 meters, it should take 5 secs if moving 1m/sec
# each info is incoming every .1 secs
width = 500
height = 500
bgcolor = 0,0,0
hz = 10.0
ms = (int)(1000/hz)
sec = ms / 1000.0

clock = pygame.time.Clock()
screen = pygame.display.set_mode((width, height))
class Navigator:
	box_color = 192, 192, 192
	def __init__(self, x, y):
		#set intial values
		self.x = x
		self.y = y
		self.w = 10
		self.h = 10
		
	def render(self, v):
		#find new position based on update frequency
		#sec = ms / 1000.0 seconds have passed since last update
		dx = v.linear.x * sec
		dy = v.linear.y * sec
		print dx,dy
		
		#update position
		self.x = self.x+dx
		self.y = self.y+dy
		
		#clear screen and redraw the location of the robot
		screen.fill(bgcolor)
		pygame.draw.rect(screen, self.box_color, (self.x,self.y,self.w,self.h), 0 )
		
		pygame.display.flip()
		#mas frames per second
        clock.tick(240)
		



if __name__ == '__main__':
    #listener()
    
    rospy.init_node('naviagtion_visualizer', anonymous=True)
    velocity = Twist()
    velocity.linear.x = 100
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    my_Navigator = Navigator(0, height/2)
    while not rospy.is_shutdown():
		my_Navigator.render(velocity)
		#delay by 100 miliseconds or 10 hz
		pygame.time.wait(ms)
    rospy.spin()
