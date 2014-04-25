#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from std_msgs.msg import Float64

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
    pub = rospy.Publisher('random_movement', Float64)
    rospy.init_node('dummy_etch_talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    lastx = 0
    lasty = 0

    while not rospy.is_shutdown():
        dx = int(displace(lastx, screen_width))
        dy = int(displace(lasty, screen_height))
        dr = (dx, dy)
        lastx += dx
        lasty += dy

        rospy.loginfo(dr)
        Float64[] msg
        msg = Float64()
        msg.data = dr
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()

