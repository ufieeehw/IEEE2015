#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('navigation_control_signals', Twist)
    rospy.init_node('navigation_controller', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        #to be replaced with more robust code
        velocity = Twist()
        velocity.linear.x = 100
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 100
        
        pub.publish(velocity)
        r.sleep()

if __name__ == '__main__':
     talker()
