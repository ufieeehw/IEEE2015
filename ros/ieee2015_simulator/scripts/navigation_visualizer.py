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
hz = 20.0
ms = (int)(1000/hz)
sec = ms / 1000.0


#sufrace around the box
navsurf = pygame.Surface((20, 20))
navsurf.fill((200, 0, 0))

#color key for blitting
navsurf.set_colorkey((255, 0, 0))

clock = pygame.time.Clock()
screen = pygame.display.set_mode((width, height))

class Navigator:
    box_color = 192, 192, 192

    def __init__(self, x, y):
        #set intial values
        self.navbox = pygame.Rect((x, y, 20, 20))
        self.degree = 1

    def render(self, v):
        #find new position based on update frequency
        #sec = ms / 1000.0 seconds have passed since last update
        dx = v.linear.x * sec
        dy = v.linear.y * sec

        #find new angle of orientation
        dtheta = 3

        #update position
        self.navbox.x += dx
        self.navbox.y += dy

        #update orientation
        self.degree += dtheta
        if self.degree > 360:
            self.degree -= 360

        #clear screen
        screen.fill(bgcolor)

        #draw the location (base) of the robot
        pygame.draw.rect(screen, self.box_color, self.navbox, 0 )

        #rotate surf
        rotatedSurf =  pygame.transform.rotate(navsurf, self.degree)

        #get the rect of the rotated surf and set it's center to the base (navbox)
        rotRect = rotatedSurf.get_rect()
        rotRect.center = self.navbox.center
        screen.blit(rotatedSurf, rotRect)

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
