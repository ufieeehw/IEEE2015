#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from std_msgs.msg import Float64
from ieee2015_simulator.msg import Float_List
screen_width = 150
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
        #dr = (dx, dy)



        lastx += dx
        lasty += dy

        #rospy.loginfo(dr)
        msg = Float_List()
        #msg.data = []
        f1 = Float64(dx)
        f2 = Float64(dy)
        #msg.data = dr
        msg.data = [f1,f2]
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
     talker()
