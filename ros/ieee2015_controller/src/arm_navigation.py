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


''' Notes '''
# Creating Grid and Arm Path Plannging are unused for chess
# Arm Movement is start of chess code
# Some values still neeed to be added  
# Things are probbaly wrong, including spelling
# Check to see if variables are correctly modified
# x is distance from base, z is height, y is base angle
# Sorry, its probably bad


''' Creating Grid '''
class Diagram(object):
    def __init__(self, arm_limits, obstacles):
        grid[x_max][z_max]
        # The 4 "corners" for the grid will be the limits of the arm
        for i in xrange (0, x_max):
            for j in xrange (0, z_max):
                if grid[i][j] == obstacle:
                    ''' Need to get obstacles from computer vision '''
                    # obstacle[i][j] # Obstacles in their own array
                    # Don't forget to add size of robot to obstacle


''' Arm Path Planning '''
# Of end-effector
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
    found_path = False

    while not frontier.empty():
        current = frontier.get()
        # Chooses starting point for loop

        if current == goal:
            found_path = True
            break
            # Ends when path is found

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            # Creates new cost for the step by adding the new step to the rest of the path
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                # Either a new coordiante or less cost
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                # Chooses next point
                came_from[next] = current

    if found_path is True:
        return came_from
        # Has the path
    else:
        return -1

def shorten_path(path):
    # Deletes extra points in a line to shorten travel
    for i in range ((len(path)) - 1):
        m = (path[i + 1][1] - path[i][1]) / (path[i + 1][0] - path[i][0])
        y1 = path[i][1]
        x2 = path[i][0]
        j = 1
        for j in range ((len(path)) - 2):
            if (path[i + 1 + j][1] - y1) == m * (path[i + 1 + j][0] - x1):
                path[i + 1][1] = -1
                path[i + 1][0] = -1


''' Arm Movement'''
# Variables
length_elbow = .148 # In meters
length_shoulder = .160 # In meters
length_end_effector = 0 ''' May not be neccesary, just add length to height ''' # In meters
joint_one_angle = '''To Add'''
joint_two_angle = '''To Add'''
distance_from_board = '''To Add'''


def begin():
    start_one = ''' To Add ''' # Need to decide defualt, in radians
    start_two = ''' To Add ''' # Need to decide defualt, in radians
    start_three = ''' To Add ''' # Need to decide defualt, in radians
    move_joint_one(start_one) ''' Need to use actual movement command '''
    move_joint_two(start_two) ''' Perferably move both at the same time '''
    move_base_angle(start_three)
    assign_joint_angles(start_one, start_two)

def get_start_cordinates():
    # Gets the current position of the end-effector using forward kinematics
    x_end_effector = length_elbow * cos(joint_one_angle) + (length_shoulder + length_end_effector) * sin(joint_two_angle)
    z_end_effector = length_elbow * sin(joint_one_angle) + (length_shoulder + length_end_effector) * cos(joint_two_angle)
    return (x_end_effector, z_end_effector)

def assign_joint_angles(joint_one, joint_two):
    # Sets the joint angles, '''Needs to''' modifies class variable 
    joint_one_angle = joint_one
    joint_two_angle = joint_two

def find_angles(x, z):
    # Finds the angles of the joints for the desired position using inverse kinematics
    c2 = (x ** 2 + z ** 2 - length_elbow ** 2 - length_shoulder ** 2)/(2 * length_elbow * length_shoulder)
    joint_one_angle = arccos(c2)
    s2 = (1 - c2 ** 2) ** .5
    joint_two_angle = arcsin((z * (length_elbow + length_shoulder * c2) - x * length_shoulder * s2)/(x ** 2 + z ** 2))
    return (joint_one_angle, joint_two_angle)

def do_move(x, y, z):
    # Actually moves to desired position
    angle_one, angle_two = find_angles(x, z)
    move_joint_one(angle_one]) ''' Need to use actual movement command '''
    move_joint_two(angle_two) ''' Perferably move both at the same time '''
    assign_joint_angles(angle_one, angle_two)


''' Chess Specific '''
def chess_square_to_coordinate(row, column):
    # Converts the given row and column of the chess piece to a polar coordinate
    square_length = .009 # In meters  
    if row > 4: # This is creating a grid with the middle of chess being 0
        row = row - 4
    else: 
         row = row - 5
    square_distance = (row ** 2 + column ** 2) ** .5
    distance = (square_length * square_distance) - (.5 * square_length) + distance_from_board # This is an attempt to convert to an actual distance and get to center of square
    base_angle = tan(column / row)

def pick_up_piece(x, y, z):
    # Picks up piece and returns to certain height, x, y, z is the original destination
    angle_one, angle_two = find_angles(x,''' To Add ''') # Secound variable will need to be adjusted for heigth
    # Open end-effector
    move_joint_one(angle_one) ''' Need to use actual movement command '''
    move_joint_two(angle_two)
    # Close end-effector
    do_move(x, y, z) 

def drop_off_piece(x, y, z):
    # Picks up piece and returns to certain height, x, y, z is the original destination
    angle_one, angle_two = find_angles(x,''' To Add ''') # Secound variable will need to be adjusted  
    # Open end-effector
    move_joint_one(angle_one) ''' Need to use actual movement commands '''
    move_joint_two(angle_two)
    # Close end-effector
    do_move(x, y, z) 

def chess_turn():
    # Does one chess turn
    begin() # Will start with default angles
    # Observe board
    # Send pieces to AI
    # AI sends row column of piece to be picked up and if spot is occupied
    if # Occupied
        do_move(x, y, z) # Move to occupied spot
        # Image
        pick_up_piece(x, y, z)
        do_move(x, y, z) # Move off board
        drop_off_piece(x, y, z)
    do_move(x, y, z) # Does move to get to above piece
    # Image
    pick_up_piece(x, y, z)
    begin() # Will start with default angles
    # AI sends row column of drop off
    do_move(x, y, z) # Does move to get to drop off
    drop_off_piece(x, y, z)
    begin() # Will start with default angles


''' Running '''
playing_chess = True
while playing_chess == True:
    chess_turn()

'''Stuff'''
# Look at publichers and subscribers
# tf 
# ros actions and services 
