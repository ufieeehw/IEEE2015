#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int64

def camera_posi_x():
    pub = rospy.Publisher('/pose', Int64, queue_size=10)
    rospy.init_node('camera_pose', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rob_pos = 6
        rospy.loginfo(rob_pos)
        pub.publish(rob_pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_posi_x()
	camera_posi_y()
	camera_orien_z()

    except rospy.ROSInterruptException:
        pass
