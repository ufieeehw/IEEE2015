#!/usr/bin/env python
import rospy
import pygame

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose, Quaternion
from tf import transformations as tf_trans

import math
import numpy as np

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

class Rover(object):
    box_color = 192, 192, 192
    def __init__(self, x, y):
        #set intial values
        self.navbox = pygame.Rect((x, y, 20, 20))
        self.degree = 1
        
        #sufrace around the box
        self.navsurface = pygame.Surface((20, 20))
        self.navsurface.fill((200, 0, 0))
        #color key for blitting
        self.navsurface.set_colorkey((255, 0, 0))

        # Add a pose publisher
        self.position = np.array([x,y], np.int32)
        self.pose_pub = rospy.Publisher('pose', PoseStamped)

    def render(self, v):
        #find new position based on update frequency
        #sec = ms / 1000.0 seconds have passed since last update
        dx = v.linear.x * sec
        dy = v.linear.y * sec

        #>
        self.position += (self.forward_vector * dx) + (self.left_vector * dy)
        
        #find new angle of orientation
        dtheta = v.angular.z * sec

        #>update position
        # self.navbox.x += dx
        # self.navbox.y += dy
        
        #> Thoughts on this, Aaron?
        # This way, we're simulating local twist instead of absolute
        # I have another idea involving reference-frame switching, that we should talk about
        # This makes the visualizer extraordinarily difficult to control by keys, though
        self.navbox.x, self.navbox.y = self.position

        #update orientation
        self.degree += dtheta
        if self.degree > 360:
            self.degree -= 360

        #draw the location (base) of the robot
        #pygame.draw.rect(screen, self.box_color, self.navbox, 0 )

        #rotate surface
        rotatedSurf = pygame.transform.rotate(self.navsurface, self.degree)

        #get the rect of the rotated surface and set it's center to the base (navbox)
        rotRect = rotatedSurf.get_rect()
        rotRect.center = self.navbox.center
        screen.blit(rotatedSurf, rotRect)
        
        # Publish position to 'pose' topic
        self.publish_pose()


    def publish_pose(self):
        '''Publish Pose
        (Sorry Aaron, couldn't make controller work without)
        '''
        _orientation = tf_trans.quaternion_from_euler(0,0,self.degree)
        self.pose_pub.publish(
            PoseStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/course',
                ),
                pose=Pose(
                    position=Point(self.navbox.x, self.navbox.x, 0.0),
                    orientation=Quaternion(*_orientation),
                )
            )
        )

    @property 
    def rad_angle(self):
        return math.radians(self.degree)

    @property
    def forward_vector(self):
        return np.array([math.cos(self.rad_angle), math.sin(self.rad_angle)])
    @property
    def left_vector(self):
        return np.array([math.cos(self.rad_angle+math.pi/2), math.sin(self.rad_angle+math.pi/2)])

        
class Course:
    point_color = 100, 50, 50
    visited_color = 100 , 0 , 0
    def __init__(self, rover, waypoints):
        self.rover = rover
        self.waypoints = waypoints
        self.target = waypoints[0]
        #I ASSUMING lists are false by default!!!!
        self.isPointVisited = []
        for w in waypoints :
            self.isPointVisited.append(False)
        
    def render_waypoints(self):
        i = 0
        for point in self.waypoints:
            if abs(point.x - self.rover.navbox.x) <= 20  and abs(point.y - self.rover.navbox.y) <= 20:
                self.isPointVisited[i] = True
            box = pygame.Rect((point.x, point.y, 20, 20))
            if self.isPointVisited[i]:
                pygame.draw.rect(screen, self.visited_color, box, 0)
            else:
                pygame.draw.rect(screen, self.point_color, box, 0)
                
            i = i + 1
                
    def render(self, rover_velocity):
        #clear screen
        screen.fill(BG_COLOR)

        #render objects on the surface
        self.rover.render(rover_velocity)
        self.render_waypoints()
        pygame.display.flip()

        #mas frames per second
        clock.tick(240)
        
if __name__ == '__main__':
    waypoint = Point()
    waypoint.x = SCREEN_WIDTH/2
    waypoint.y = SCREEN_HEIGHT/2
    waypoint.z = 0
    test_waypoints = [waypoint]
    
    main_Rover = Rover(0, SCREEN_HEIGHT/2)
    main_Course = Course(main_Rover, test_waypoints)
    
    #listener initalizations
    rospy.init_node('navigation_visualizer', anonymous=True)
    #when a message is recieved the main_Course's render function will be called
    rospy.Subscriber("navigation_control_signals", Twist, main_Course.render)
    rospy.spin()

