#!/usr/bin/env python
import os
import sys
import rospy
import pygame
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose, Quaternion
scriptpath = "navigation_visualizer.py"

# Add the directory containing your module to the Python path (wants absolute paths)
sys.path.append(os.path.abspath(scriptpath))

# Do the import
import navigation_visualizer as nv

#feet per second
SAFE_SPEED = 2

#iterations per second
hertz = 60
    
class Waypoint_Handler(object):
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.pose = Pose(
                position = Point(),
                orientation = Quaternion(), #Radians
        )
        self.target = waypoints[0]
        #false by default
        self.isPointVisited = []
        for w in waypoints:
            self.isPointVisited.append(False)
            
        # Add a twist publisher
        self.twist_pub = rospy.Publisher('automatic_navigation_twists', Twist)
            
    def set_pose(self, pose_stamped) :
         self.pose = pose_stamped.pose
         
    def check_waypoints(self):
        i = 0
        rover_rect = pygame.Rect(0, 0, 50, 50)
        rover_rect.center = (self.pose.position.x, self.pose.position.y)
        
        for waypoint in self.waypoints:
            boundary = pygame.Rect((0, 0, 100, 100))
            boundary.center = (waypoint.x, waypoint.y)
            if boundary.colliderect(rover_rect):
                if not self.isPointVisited[i]:
                    self.isPointVisited[i] = True
                    print "REACHED WAYPOINT at " 
                    print self.pose.position
                    if i < len(self.waypoints)-1 and waypoint.x == self.target.x and waypoint.y == self.target.y :
                        self.target = self.waypoints[i+1]
            i = i + 1
            
            self.move_to_target()
            
    def move_to_target(self):
        #This need to be changed to be the robot's current velocity, then be changed by a safe acceleration
        new_velocity = Twist()
        #get direction, 0 means we're here
        #this method is temporary and not ideal, SAFE_SPEED must be accelerated to
        dx = self.pose.position.x - self.target.x
        if dx > 0:
            new_velocity.linear.x = -SAFE_SPEED
        elif dx < 0:
            new_velocity.linear.x = SAFE_SPEED
        else:
            new_velocity.linear.x = 0
            
        dy = self.pose.position.y - self.target.y
        if dy > 0:
            new_velocity.linear.y = -SAFE_SPEED
        elif dy < 0:
            new_velocity.linear.y = SAFE_SPEED
        else:
            new_velocity.linear.y = 0
        
        self.twist_pub.publish(new_velocity)

if __name__ == '__main__':
    handler = Waypoint_Handler(nv.waypoint_list)
    
    #listener initalizations
    rospy.init_node('waypoint_handler', anonymous=True)
    rospy.Subscriber('pose', PoseStamped, handler.set_pose)
    
	#initalizations for pygame
    clock = pygame.time.Clock()
    
    while not rospy.is_shutdown():
        handler.check_waypoints()
        #mas frames per second
        clock.tick(hertz)
      
