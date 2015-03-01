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


def chess_defualt():
    # Set the default area above chess board
    do_move(default_x, default_y, default_z) ''' Add Defaults'''

def do_move(x, y, z):
    # Actually moves to desired position
    self.coordiantes_pub = rospy.Publisher('/arm_des_pose', PointStamped, self.got_des_pose, queue_size=1)
    self.shoulder_pub.publish(Float64(x, y, z)) # May not be right

def polar_to_cartesian(x, y, z):
    # Converts the foward dstance, base angle, and height to cartesian
    distance = x
    angle = y
    x = distance * cos(angle)
    y = distance * sin(angle)
    return (x, y, z) # 3D cartetsian coordiantes 

def chess_square_to_coordinate(row, column):
    # Converts the given row and column of the chess piece to a polar coordinate
    if ((row > 8) or (column > 8)) and ((row != 9) or (column != 4)):# Designated drop of spot
        print("Excceds chess board size")
        return None
    square_length = .009 # In meters  
    if row > 4: # This is creating a grid with the middle of chess being 0
        row = row - 4
    else: 
         row = row - 5
    square_distance = (row ** 2 + column ** 2) ** .5
    distance = (square_length * square_distance) - (.5 * square_length) + distance_from_board # This is an attempt to convert to an actual distance and get to center of square
    base_angle = tan(column / row)
    return (distance, base_angle)

def pick_up_piece(x, y, z): 
    # Picks up piece and returns to certain height, x, y, z is the original destination
    ## Take piece picture
    ## Transformation from piece
    ## Open end-effector
    do_move(x, y, ''' To Add ''') # Moves down certain height 
    ## Close end-effector
    do_move(x, y, z) 

def drop_off_piece(x, y, z): 
    # Picks up piece and returns to certain height, x, y, z is the original destination
    do_move(x, y, ''' To Add ''') # Moves down certain height 
    ## Open end-effector
    do_move(x, y, z) 
    ## Close end-effector

def chess_move():
    # May use ROS actons. Does movemenets for chess, open to change
    chess_default()
    ## Take full chess picture
    self.pose_sub = rospy.Subscriber('/chess_turn', , queue_size=1)# Recive row, column, and if picking up or dropping off
    (x, y) = chess_square_to_coordinate(row, column)
    polar_to_cartesian(x, y, z = ''' To Add ''')# Moves down certain height 
    do_move(x, y, z)
    if :# Pick up
        pick_up_piece(x, y, z)
    else :# Drop off
        drop_off_piece(x, y, z)
    chess_default()


'''Stuff'''
# Look at publishers and subscribers
# tf 
# ros actions and services 
# Ros errors
# Py lint
# Get the off set from the chess board to center of robot 
# From camera to piece to end effector
# ieee_navigation for astar look into ros packages
