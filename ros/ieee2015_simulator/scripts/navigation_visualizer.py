#!/usr/bin/env python
import rospy
import pygame
import random
import os

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose, Quaternion
from tf import transformations as tf_trans

import math
import numpy as np

#constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 400
WAYPOINT_LENGTH = 100
BG_COLOR = 0,0,0

PXL_PER_METER = 15  # Or something like that

#global variables
fps = 60.0

#initalizations for pygame
clock = pygame.time.Clock()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

def load_image(name, colorkey=False):
    name = os.path.join(os.getcwd(), 'data', name)
    try:
        image = pygame.image.load(name)
        colorkey = image.get_at((0, 0))
        if colorkey is True:
            image.set_colorkey(colorkey, pygame.RLEACCEL)
    except:
        print 'Unable to load: ' + name
    return image.convert_alpha() #Convert any transparency in the image
    
class Rover(object):
    box_color = 192, 192, 192
    def __init__(self, x, y):
        #set intial values
        self.navbox = pygame.Rect((x, y, 50, 50))
        self.degree = 1
        self.velocity = Twist()
        
        #this may not be nessesary, playing it safe
        self.velocity.linear.x = 0
        self.velocity.linear.y = 0
        self.velocity.linear.z = 0
        self.velocity.angular.x = 0
        self.velocity.angular.y = 0
        self.velocity.angular.z = 0
        
        #sufrace around the box
        self.navsurface = pygame.Surface((50, 50))
        self.navsurface = load_image('rover.png')
        #color key for blitting
        self.navsurface.set_colorkey((255, 0, 0))

        # Add a pose publisher
        self.position = np.array([x,y], np.float32)
        self.pose_pub = rospy.Publisher('pose', PoseStamped)

    def reposition(self):
        #find new position based on update frequency
        #dt seconds have passed since last frame update
        dt = 1.0/fps
        dx = self.velocity.linear.x * dt * PXL_PER_METER
        dy = self.velocity.linear.y * dt * PXL_PER_METER

        self.position += (self.forward_vector * dx) + (self.left_vector * dy)
        
        self.navbox.x, self.navbox.y = self.position

        #find new angle of orientation
        dtheta = math.degrees(self.velocity.angular.z) * dt
        
        #update orientation
        self.degree += dtheta
        if self.degree > 360:
            self.degree -= 360
        elif self.degree < 0:
            self.degree += 360
        #draw the location (base) of the robot
        #pygame.draw.rect(screen, self.box_color, self.navbox, 0 )

        #rotate surface
        self.navsurf = pygame.transform.rotate(self.navsurface, self.degree)
        #get the rect of the rotated surface and set it's center to the base (navbox)
        rotRect = self.navsurf.get_rect()
        rotRect.center = self.navbox.center
        self.navbox = rotRect
        
        # Publish position to 'pose' topic
        self.publish_pose()
        
    def set_velocity(self, v):
        self.velocity = v
        
    def render(self):
        self.reposition()
        screen.blit(self.navsurface, self.navbox)

    def publish_pose(self):
        '''Publish Pose
        (Sorry Aaron, couldn't make controller work without)
        '''
        _orientation = tf_trans.quaternion_from_euler(0,0,self.rad_angle)
        self.pose_pub.publish(
            PoseStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/course',
                ),
                pose=Pose(
                    position=Point(self.position[0], self.position[1], 0.0),
                    orientation=Quaternion(*_orientation), # Radians
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
            box = pygame.Rect((0, 0, 20, 20))
            box.center = (point.x, point.y)
            boundary = pygame.Rect((point.x, point.y, WAYPOINT_LENGTH, WAYPOINT_LENGTH))
            boundary.center = (point.x, point.y)
            if boundary.colliderect(self.rover.navbox):
                self.isPointVisited[i] = True
            
            if self.isPointVisited[i]:
                pygame.draw.rect(screen, self.visited_color, boundary, 0)
            else:
                pygame.draw.rect(screen, (250,250,250), boundary, 0)
            pygame.draw.rect(screen, self.point_color, box, 0)
                
            i = i + 1
                
    def render(self):
        #clear screen
        screen.fill(BG_COLOR)

        #render objects on the surface
        self.render_waypoints()
        self.rover.render()
        pygame.display.flip()

        #mas frames per second
        clock.tick(fps)
        
    def callback(self, rover_velocity):
        self.rover.set_velocity(rover_velocity)
        
if __name__ == '__main__':
    random_waypoints = []
    for i in range(3):
        waypoint = Point()
        waypoint.x = random.randint(WAYPOINT_LENGTH/2, SCREEN_WIDTH - WAYPOINT_LENGTH/2)
        waypoint.y = random.randint(WAYPOINT_LENGTH/2, SCREEN_HEIGHT - WAYPOINT_LENGTH/2)
        waypoint.z = 0
        random_waypoints.append(waypoint)
       

    main_Rover = Rover(0, SCREEN_HEIGHT/2)
    main_Course = Course(main_Rover, random_waypoints)
    
    #listener initalizations
    rospy.init_node('navigation_visualizer', anonymous=True)
    #when a message is recieved the main_Course's render function will be called
    rospy.Subscriber("navigation_control_signals", Twist, main_Course.callback)
    
    while not rospy.is_shutdown():
        main_Course.render()
    rospy.spin()

