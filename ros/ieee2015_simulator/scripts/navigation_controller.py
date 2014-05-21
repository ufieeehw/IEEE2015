#!/usr/bin/env python
import rospy
import pygame
from geometry_msgs.msg import Twist

#constants
SCREEN_WIDTH = 200
SCREEN_HEIGHT = 200
bgcolor = 256,256,256

def talker():
    pub = rospy.Publisher('navigation_control_signals', Twist)
    rospy.init_node('navigation_controller', anonymous=True)
    r = rospy.Rate(10) # 10hz
    velocity = Twist()
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    increment = 20
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RIGHT:
                    velocity.linear.x += increment
                if event.key == pygame.K_LEFT:
                    velocity.linear.x -= increment
                if event.key == pygame.K_UP:
                    velocity.linear.y -= increment
                if event.key == pygame.K_DOWN:
                    velocity.linear.y += increment
                if event.key == pygame.K_a:
                    velocity.angular.z -= increment
                if event.key == pygame.K_d:
                    velocity.angular.z += increment
    
        pub.publish(velocity)
        r.sleep()

if __name__ == '__main__':
     talker()
