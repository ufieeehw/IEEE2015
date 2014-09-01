#!/usr/bin/env python
import rospy
import pygame
from geometry_msgs.msg import Twist

#constants
SCREEN_WIDTH = 200
SCREEN_HEIGHT = 200
def talker():
    #initalizations
    pub = rospy.Publisher('desired_velocity', Twist)
    rospy.init_node('navigation_controller', anonymous=True)
    r = rospy.Rate(10) # 10hz
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    
    #initilize velocity
    velocity = Twist()
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    
    #determines size of change when key events are recieved
    increment = 2
    degree_increment = .1
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RIGHT:
                    velocity.linear.x += increment
                if event.key == pygame.K_LEFT:
                    velocity.linear.x -= increment
                if event.key == pygame.K_UP:
                    velocity.linear.y += increment
                if event.key == pygame.K_DOWN:
                    velocity.linear.y -= increment
                if event.key == pygame.K_a:
                    velocity.angular.z += degree_increment
                if event.key == pygame.K_d:
                    velocity.angular.z -= degree_increment
    
        pub.publish(velocity)
        r.sleep()

if __name__ == '__main__':
     talker()
