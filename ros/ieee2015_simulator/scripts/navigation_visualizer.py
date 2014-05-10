#!/usr/bin/env python
# Software License Agreement (BSD License)
import rospy
import pygame
from geometry_msgs.msg import Twist

class Course:
	def __init__(self, length, width):	
		#dimensions in meters
		self.length = length
		self.width = width
		
class Navigator:
	def __init__(self, course, x, y, twist):
		self.couse = course
		self.x = x
		self.y = y
		self.velocity = twist


if __name__ == '__main__':
    #listener()
    
    rospy.init_node('naviagtion_visualizer', anonymous=True)
    velocity = Twist()
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    my_Course = Course(10,10)
    my_Navigator = Navigator(my_Course, 0, 0, velocity)
    rospy.spin()
