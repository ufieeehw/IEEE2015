#!/usr/bin/python
## Math
import numpy as np
## Display
import pygame
import time
## Ros
import rospy
from tf import transformations as tf_trans
## Ros Msgs
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion


''' Major work in progress '''


''' Modifying arm controller '''
class arm(SCARA_Controller):
	# Moddified from SCARA_Controller in arm_controller to include thrid axis
	def __init__(self, lengths=(100, 100)):
        self.platform_pub = rospy.Publisher('arm_platform_angle', Float32)
   
    def got_des_pose(self, msg):
        # Get desired position
        self.des_position = np.array([msg.point.x, msg.point.y, msg.point.z])
        if solutions is not None:
            print("Targeting arm-base local position {} (m)".format(self.des_position))
            base, elbow, platform = solutions
            self.publish_angles(base, elbow, platfrom) 

    def publish_angles(self, base, elbow, platform):
        # Publishes the angles
        print("Targeting angles base: {0:0.2f} (rad), elbow: {0:0.2f} (rad), platform: {0:0.2f} (rad)".format(base, elbow, platform))
        self.platform_pub.publish(Float32(data=platform))  

	def solve_angles(self, pt):
        # Need to figure out how platform rotation factors in
        x, y, z = pt

        distance = np.sqrt((x**2) + (y**2))
        if distance > (self.length_1 + self.length_2):
            print("Target arm position out of bounds")
            return None

        base_angle = np.arctan2(y, x) - np.arccos(distance / (2 * self.length_1))
        abs_joint_angle = np.arctan2(y - (self.length_1 * np.sin(base_angle)),
                                     x - (self.length_1 * np.cos(base_angle)))
        joint_angle = abs_joint_angle - base_angle
        return (normalize_angle(base_angle + np.pi), normalize_angle(np.pi + joint_angle))
  


''' Creating Grid for path planning '''
class Diagram(object):
    def __init__(self, arm_limits, obstacles):
	   grid[x_max][z_max]
	   # The 4 "corners" for the grid will be the limits of the arm
	   for i in xrange (0, x_max):
            for j in xrange (0, x_max):
              if grid[i][j] == obstacle:
                   ''' Need to get obstacles from computer vision '''
                   grid[i][j] = 999 # Some way to differentiate the obstacle
                  # Don't forget to add size of robot to obstacle




''' Arm path planning '''
from Queue import PriorityQueue
# Used to organize items by priority 

def heuristic(a, b):
    (x1, z1) = a
    (x2, z2) = b
    return abs(x1 - x2) + abs(z1 - z2)
# Helps, give a priority to the right direction

def a_star_search(graph, start, goal):
	# Start is current location of end-effector and goal is the destination of end-effector
    frontier = PriorityQueue()
    # Frontier being edge of explored coordinates 
    frontier.put(start, 0)
    came_from = {}
    # Keeps track of previous steps
    cost_so_far = {}
    # Cost of previous steps, currently just distance traveled
    came_from[start] = None
    cost_so_far[start] = 0
   
    while not frontier.empty():
        current = frontier.get()
        # Chooses starting point for loop
      
        if current == goal:
            break
            # Ends when path is found
      
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            # Crates new cost for the step by adding the new step to the rest of the path
            if next not in cost_so_far or new_cost < cost_so_far[next]:
            	# Either a new coordiante or less cost
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                # Chooses next point
                came_from[next] = current
   
    return came_from
    # Has the path

''' Need to account for no path found '''
''' Maybe account for exces waypoints '''

''' Example implementation:
came_from, cost_so_far = a_star_search(diagram, (1, 4), (7, 8))
draw_grid(diagram, width=3, point_to=came_from, start=(1, 4), goal=(7, 8))
print()
draw_grid(diagram, width=3, number=cost_so_far, start=(1, 4), goal=(7, 8))
''' 
''' Need to create diagaram with obstacles for this implementation '''


# Path finding if for end effector
# Includes boundaries for obstacles + size of arm
# Finds position of endeffector with forward kinematics
# After finding path
# Use inverese kinematics to discover the needed angles along that path
# Still need to work out Jacobian Matrixes


''' Arm movment'''

def get_start:
    # Gets the current position of the end-effector using forward kinematics
    x_end_effector = length_upper_arm * cos(joint_one_angle) + (length_lower_arm + length_end_effector) * sin(joint_two_angle)
    z_end_effector = length_upper_arm * sin(joint_one_angle) + (length_lower_arm + length_end_effector) * cos(joint_two_angle)
    start = (x_end_effector, z_end_effector)
    return start

def find_angles(x, z):
    # Finds the angles of the joints for the desired position using inverse kinematics
    c2 = (x ** 2 + z ** 2 - length_upper_arm ** 2 - length_lower_arm ** 2)/(2 * length_upper_arm * length_lower_arm)
    joint_one_angle = arccos(c2)
    s2 = (1 - c2 ** 2) ** .5
    joint_two_angle = arcsin((z * (length_upper_arm + length_lower_arm * c2) - x * length_lower_arm * s2)/(x ** 2 + z ** 2))
    angles = (joint_one_angle, joint_two_angle)
    return angles

def do_move(x, z):
    # Actually moves to desired position
    angles = find_angles(x, z)
    move_joint_one(angles[0])
    move_joint_two(angles[1])
    ''' Need to use actually command '''
    ''' Perferably move both at the same time '''


''' Need to establish lengths, and find way to discover angles of joints '''


''' Running '''

get_des_pose(self, msg)
# Get desired pose from chess game or other objective

rotate_arm_base(self, z)
# Rotates base to be inline with target

diagram = Diagram()

get_init_pose(self, lengths=(100, 100))
# Now treated like a single plane

possibe_reach = distance(self, pt)
# Gets distance between destination and initial pose

if possibe_reach > 0:
	path = a_star_search(diagram, get_start, get_des_pose(self, msg))
    for xrange in path:
        do_move(path)