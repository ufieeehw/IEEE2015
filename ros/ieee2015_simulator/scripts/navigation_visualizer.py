#!/usr/bin/env python
import rospy
import pygame
import random
import os, sys

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose, Quaternion
from tf import transformations as tf_trans

import math
import numpy as np

#constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 400
WAYPOINT_LENGTH = 100
BG_COLOR = 200,200,100

PXL_PER_METER = 15  # Or something like that

#global variables
fps = 60.0


#waypoint are shared with other files
waypoint_list = [Point(400, -200, 0), Point(600, -200, 0), Point(0, -300, 0)]

def load_image(name, colorkey=False):
    #get file directory
    sim_folder = os.path.dirname(os.path.dirname(sys.argv[0]))
    name = os.path.join(sim_folder, 'data', name)
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
        self.rover_rect = pygame.Rect((0, 0, 50, 50))
        self.rover_rect.center = (x, y)
        #image surface around the box
        self.master_image = pygame.Surface((50, 50))
        self.master_image = load_image('rover.png')
        self.rover_image = self.master_image
        #self.rover_rect = self.rover_image.get_rect()
        
        #set intial values
        self.direction = 0
        self.velocity = Twist()
        
        #color key for blitting
        self.rover_image.set_colorkey((0, 0, 0))

        # Add a pose publisher
        self.position_vector = np.array([x, -y], np.float32)
        self.pose_pub = rospy.Publisher('pose', PoseStamped)

    def reposition(self):
        #find new position based on update frequency
        #dt seconds have passed since last frame update
        dt = 1.0/fps
        dx = self.velocity.linear.x * dt * PXL_PER_METER
        dy = self.velocity.linear.y * dt * PXL_PER_METER

        self.position_vector += (self.forward_vector * dx) + (self.left_vector * dy)
        
        self.rover_rect.x, self.rover_rect.y = self.position_vector[0], -self.position_vector[1]

        #find new angle of orientation
        dtheta = math.degrees(self.velocity.angular.z) * dt
        
        #update direction
        self.direction += dtheta
        self.direction % 360
        
        #old_center = self.rover_rect.center
        #self.rover_image = pygame.transform.rotate(self.master_image, self.direction)
        #self.rover_rect = self.rover_image.get_rect()
        #self.rover_rect.center = old_center
        #old_center = self.rover_rect.center
        
    
        #rotate surface
        self.rover_image = pygame.transform.rotate(self.master_image, self.direction)
        
        #get the rect of the rotated surface and set it's center to the base (self.rover_rect)
        self.rover_rect = self.rover_image.get_rect(center = self.rover_rect.center) 
        
        # Publish position to 'pose' topic
        self.publish_pose()
        
    def set_velocity(self, v):
        self.velocity = v
        
    def render(self):
        self.reposition()
        screen.blit(self.rover_image, (self.rover_rect.center[0]-(self.rover_rect.width/2), self.rover_rect.center[1]-(self.rover_rect.height/2)))
        
    def publish_pose(self):
        '''Publish Pose
        '''
        _orientation = tf_trans.quaternion_from_euler(0,0,self.rad_angle)
        self.pose_pub.publish(
            PoseStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/course',
                ),
                pose = Pose(
                    position = Point(self.position_vector[0], self.position_vector[1], 0.0),
                    orientation = Quaternion(*_orientation), #Radians
                )
            )
        )

    @property 
    def rad_angle(self):
        return math.radians(self.direction)

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
        #false by default
        self.isPointVisited = []
        for w in waypoints :
            self.isPointVisited.append(False)
        
    def render_waypoints(self):
        i = 0
        for point in self.waypoints:
            box = pygame.Rect((0, 0, 20, 20))
            box.center = (point.x, -point.y)
            boundary = pygame.Rect((point.x, -point.y, WAYPOINT_LENGTH, WAYPOINT_LENGTH))
            boundary.center = (point.x, -point.y)
            if boundary.colliderect(self.rover.rover_rect):
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
        ###event handling###
        ev = pygame.event.get()
        for event in ev:
            #get the position when mouse is released, so it can be published
            if event.type == pygame.MOUSEBUTTONUP:
                main_Rover.publish_pose()
        #mas frames per second
        clock.tick(fps)
        
    def callback(self, rover_velocity):
        self.rover.set_velocity(rover_velocity)
        
if __name__ == '__main__':
    
    #initalizations for pygame
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    main_Rover = Rover(0, SCREEN_HEIGHT/2)
    main_Course = Course(main_Rover, waypoint_list)
    
    #listener initalizations
    rospy.init_node('navigation_visualizer', anonymous=True)
    
    #when a message is recieved the main_Course's render function will be called
    rospy.Subscriber("desired_velocity", Twist, main_Course.callback)
    #rospy.Subscriber("automatic_navigation_twists", Twist, main_Course.callback)
    
    while not rospy.is_shutdown():
        
        main_Course.render()
      
        
          
        rospy.spin()

