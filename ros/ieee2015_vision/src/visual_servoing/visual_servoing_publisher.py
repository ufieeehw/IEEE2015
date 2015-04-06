#!/usr/bin/python
import rospy
from visual_servoing import distance_client
#from object_detection import cardDetection
from ros_image_tools import Image_Subscriber
import cv2
from geometry_msgs.msg import PointStamped
'''How to use this:

roslaunch ieee2015_launch base_camera.launch
rosrun ieee2015_vision test_vision.py


Remove: waitKey(0) --> Blocking
Remove: Prints, use rospy.loginfo or rospy.logwarn
Remove: Things that run when imported
Remove: Extraneous imshow
Remove: Uses of cv instead of cv2 (dependency issue)

'''


def image_callback(image):
    cv2.imshow("Input Image", image)
    # find_rubix(image, 0.3)
    distance_result = distance_client.main(image)
    if (distance_result == None):
        return
    r = rospy.Rate(5)
    target_pt = distance_result.dist
    des_pose.publish(target_pt)
    #cardDetection.getCardLoc(image)
    cv2.waitKey(1)
    r.sleep()
    #return dist
    print 'something'

if __name__ == '__main__':
    rospy.init_node('test_vision')
    
    des_pose = rospy.Publisher('/robot/arm_des_pose', PointStamped, queue_size=1)

    image = Image_Subscriber('/robot/arm_camera/image_raw', image_callback)
    
    rospy.spin()