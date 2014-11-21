#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64, Float32

class Gazebo_Transformer(object):
    def __init__(self):
        print("Running transform node")
        rospy.init_node('gazebo_transformer')

        self.elbow_sub = rospy.Subscriber('/elbow_controller/command', Float64, self.convertElbow)
        self.elbow_pub = rospy.Publisher('robot/elbow_controller_gazebo/command', Float64, queue_size=2)

        self.shoulder_sub = rospy.Subscriber('/shoulder_controller/command', Float64, self.convertShoulder)
        self.shoulder_pub = rospy.Publisher('robot/shoulder_controller_gazebo/command', Float64, queue_size=2)


    def convertElbow(self, angle): 
        #_elbow_angle = np.pi - (elbow + elbow_angle_offset) data received, revert the offset
        global gl_angle
        gl_angle = angle
        elbow_angle_offset = 1.75
        elbow = np.pi - angle.data - elbow_angle_offset #remove offset
        elbow = elbow - np.pi #Before 0 degrees = 3.14, now 0 degrees = 0
        elbow = -elbow #Make below horizon positive, that is what the joint uses
        
        elbow = shoulder_angle + elbow #Find the angle relative to the shoulder. I did the trig, don't worry :P

        print("Publishing elbow angle: {}".format(elbow))
        self.elbow_pub.publish(data = elbow)


    def convertShoulder(self, angle):
        #_shoulder_angle = shoulder + shoulder_angle_offset, revert the offset
        shoulder_angle_offset = 0.3 - np.pi/2
        shoulder = angle.data - shoulder_angle_offset
        global shoulder_angle
        shoulder_angle = shoulder #To be used to find the elbow angle

        print("Publishing shoulder angle: {}".format(shoulder))
        self.shoulder_pub.publish(data = -shoulder) 
        self.convertElbow(gl_angle) #Make sure Elbow has the up to date shoulder_angle 


if __name__ == '__main__':
    Transformer = Gazebo_Transformer()
    rospy.spin()

