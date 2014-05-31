#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Float64
from ieee2015_simulator.msg import Float_List
screen_width = 250
screen_height = 1300/8

def displace(start, boundary):
    midpoint = boundary / 2
    
    #set direction
    if start < midpoint / 2 :
        direction = 1
    elif start > midpoint + midpoint / 2 :
        direction = -1
    else :
        direction = 1 if random.randint(0,1) else -1

    return direction * random.random() * midpoint / 2


def talker():
    pub = rospy.Publisher('random_movement', Float_List)
    rospy.init_node('dummy_etch_talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    lastx = screen_width/2.0
    lasty = screen_height/2.0
    
    while not rospy.is_shutdown():
        dx = int(displace(lastx, screen_width))
        dy = int(displace(lasty, screen_height))
        lastx += dx
        lasty += dy

        msg = Float_List()
        msg.float_list = [Float64(dx),Float64(dy)]
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
     talker()
