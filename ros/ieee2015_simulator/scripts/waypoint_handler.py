#!/usr/bin/env python
import os
import sys
import rospy
import pygame
import numpy as np
from std_msgs.msg import Header
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
        self.vehicle_pose = Pose(
                position = Point(),
                orientation = Quaternion(), #Radians
        )
        self.target = waypoints[0]
        #false by default
        self.isPointVisited = []
        for w in waypoints:
            self.isPointVisited.append(False)
            
        #add a PoseStamped publisher
        self.pose_pub = rospy.Publisher('desired_pose', PoseStamped)
            
    def set_vehicle_pose(self, pose_stamped) :
         self.vehicle_pose = pose_stamped.pose
         
    def check_waypoints(self):
        i = 0
        vehicle_rect = pygame.Rect(0, 0, 50, 50)
        vehicle_rect.center = (self.vehicle_pose.position.x, self.vehicle_pose.position.y)
        
        for waypoint in self.waypoints:
            boundary = pygame.Rect((0, 0, 25, 25))
            boundary.center = (self.target.x, self.target.y)
            if boundary.colliderect(vehicle_rect):
                if not self.isPointVisited[i]:
                    self.isPointVisited[i] = True
                    print "Reached desired waypoint at " 
                    print self.vehicle_pose.position
                    if i < len(self.waypoints)-1:
                        self.target = self.waypoints[i+1]
                        
            i = i + 1
            
            dx = self.target.x - self.vehicle_pose.position.x
            if dx > 0:
                target_orientation = Quaternion(
                    z = -np.sqrt(.5),
                    w = np.sqrt(.5)
                    )
            else :
                target_orientation = Quaternion(
                    z = np.sqrt(.5),
                    w = np.sqrt(.5)
                    )
            
            #publish the desired pose for the vechile
            self.pose_pub.publish(
                PoseStamped(
                    header = Header(
                        stamp=rospy.Time.now(),
                        frame_id='/course',
                    ),
                    pose = Pose(
                        position = self.target,
                        #placeholder orientation
                        orientation = target_orientation
                    )
                )
            )
                
        
if __name__ == '__main__':
    handler = Waypoint_Handler(nv.waypoint_list)
    
    #listener initalizations
    rospy.init_node('waypoint_handler', anonymous=True)
    rospy.Subscriber('pose', PoseStamped, handler.set_vehicle_pose)
    
    #initalizations for pygame
    clock = pygame.time.Clock()
    
    while not rospy.is_shutdown():
        handler.check_waypoints()
        #mas frames per second
        clock.tick(hertz)
      
