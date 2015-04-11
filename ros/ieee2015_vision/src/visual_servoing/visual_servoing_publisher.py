#!/usr/bin/python
import rospy
import numpy as np
from visual_servoing import distance_client
#from object_detection import cardDetection
from ros_image_tools import Image_Subscriber
import cv2
from geometry_msgs.msg import PointStamped
'''How to use this:
Make sure it is in the correct name space sim or robot
roslaunch ieee2015_launch arm_sim.launch or roslaunch actual arm controller


roslaunch ieee2015_launch arm_camera.launch
rosrun ieee2015_vision distance_server.py
rosrun ieee2015_vision visual_servoing_publisher.py


Remove: waitKey(0) --> Blocking
Remove: Prints, use rospy.loginfo or rospy.logwarn
Remove: Things that run when imported
Remove: Extraneous imshow
Remove: Uses of cv instead of cv2 (dependency issue)

'''


def image_reader(image):
    cv2.imshow("Input Image", image)
    # find_rubix(image, 0.3)
    distance_result = distance_client.main(image)
    
    if (distance_result == None):
        return

    r = rospy.Rate(25)
    target_pt = distance_result.dist
    print 'vector saftey', (np.linalg.norm([target_pt.point.x, target_pt.point.y]))
    if (np.linalg.norm([target_pt.point.x, target_pt.point.y]) >= 0.31):
        return 
    des_pose.publish(target_pt)
    #cardDetection.getCardLoc(image)
    cv2.waitKey(1)
    r.sleep()
    #return dist
    print 'something'

if __name__ == '__main__':
    rospy.init_node('test_vision')
    print 1
    
    des_pose = rospy.Publisher('/robot/arm_des_pose', PointStamped, queue_size=1)
    print 2
    image = Image_Subscriber('/robot/arm_camera/image_raw', image_reader)
    print 3
    #image_reader(image)
    rospy.spin()