#!/usr/bin/env python
import rospy
import pygame
from geometry_msgs.msg import Twist

#constants
SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
BG_COLOR = 0,0,0

#global variables
hz = 20.0
ms = (int)(1000/hz)
sec = ms / 1000.0

#initalizations for pygame
clock = pygame.time.Clock()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

class Navigator:
    box_color = 192, 192, 192

    def __init__(self, x, y):
        #set intial values
        self.navbox = pygame.Rect((x, y, 20, 20))
        self.degree = 1
        
        #sufrace around the box
        self.navsurf = pygame.Surface((20, 20))
        self.navsurf.fill((200, 0, 0))
        #color key for blitting
        self.navsurf.set_colorkey((255, 0, 0))
        
        #listener initalizations
        rospy.init_node('navigation_visualizer', anonymous=True)
        rospy.Subscriber("navigation_control_signals", Twist, self.callback)

    def render(self, v):
        #find new position based on update frequency
        #sec = ms / 1000.0 seconds have passed since last update
        dx = v.linear.x * sec
        dy = v.linear.y * sec

        #find new angle of orientation
        dtheta = v.angular.z * sec

        #update position
        self.navbox.x += dx
        self.navbox.y += dy

        #update orientation
        self.degree += dtheta
        if self.degree > 360:
            self.degree -= 360

        #clear screen
        screen.fill(BG_COLOR)

        #draw the location (base) of the robot
        pygame.draw.rect(screen, self.box_color, self.navbox, 0 )

        #rotate surface
        rotatedSurf =  pygame.transform.rotate(self.navsurf, self.degree)

        #get the rect of the rotated surface and set it's center to the base (navbox)
        rotRect = rotatedSurf.get_rect()
        rotRect.center = self.navbox.center
        screen.blit(rotatedSurf, rotRect)

        pygame.display.flip()

        #mas frames per second
        clock.tick(240)
        
    def callback(self, twist):
        self.render(twist)
        
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
    my_Navigator = Navigator(0, SCREEN_HEIGHT/2)
    rospy.spin()
