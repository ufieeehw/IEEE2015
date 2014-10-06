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

''' Major work in progress'''

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
      
def check_neighbours(grid[x][y]):
	# Gathers neighbour numbers
	neighbour[2]
	if y < 100:	
		neighbour [0] = grid[x][y+1]
	else
		neighbour [0] = -1
	if x is not 0:
		neighbour [1] = grid[x-1][y]
	else
		neighbour [1] = -1
	return neighbour

get_des_pose(self, msg)
# Get desired pose from chess game or other objective

rotate_arm_base(self, z)
# Rotates base to be inline with target

get_init_pose(self, lengths=(100, 100))
# Now treated like a single plane

possibe_reach = distance(self, pt)
# Gets distance between destination and initial pose

if possibe_reach > 0:
	# Max distance as in farthes arm can reach
	# Setting up grid for path finding, covers max arm movment  
	nothing = 0
	boundary = 999
	robot = 998
	goal = 1
	grid[100][100] # Can be changed for more detailed movement 

	# Anyalize area, mapping boundaries(accounting for arm) and obstacles 
	# Assign target and robot location on grid
	'''Requires Computer Vison and Slam to create boundaries'''

	path_found = False
	x = 0
	y = 0

	while path_found is False:

		while x < 100:
			# Start at [0][0]
			if (grid[x][y] is 999):
				# Ignore this node, go to next node B

			elif (grid[x][y] is 998 and neighbour[1] > 0) or (grid[x][y] > 0 and neighbour[0] is  998):
				# Path found
				# Find the boundary noThank yode with the smallest number
				path_found = True

			elif neighbour[0] is 1 or neighnour[1] is 1:
				# For starting trail
				grid[x][y] = 2

			else:
				# For creating a trail
				if neighbour[0] > 1:
					grid[x][y] = neighbour[0] + 1

				if neighbour[1] > 1:
					grid[x][y] = neighbour[1] + 1
			x++
		
		y++

		if (x > 100 and y > 100) and path_found is False:
			# Possible no solution 
			break

else # See if whole robot can move towards destination
	# Move robot and restart check
	
if path_found is True:
	# Arm moves to lowest number



'''Need to add destination position, robot, and boundaries to grid'''