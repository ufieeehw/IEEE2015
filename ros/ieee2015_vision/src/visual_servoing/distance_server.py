#!/usr/bin/env python
import tf
import rospy
import geometry_msgs.msg
import math
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point


#takes two points and a height of an object
####point1 #center point (0,0,0)
####point2 #featured point (x,y,h) h=hight of feature in meters
#looks up the transform from world(/course) to the downward facing camera(/end_camera)
#computes the real world distance between the two points by applying a camera specific constant(pixles/meters) derived from experimenting with the camera 



def handle_two_points(req):
    '''
        Real_World_width =  distance_camera_to_object * Pixel_width / constant 

    Calculate real world coordinates of desired camera location in the robot frame
    8.79167 is a constant calculated from experimenting with the camera fogal length
    distance_camera_to_object = end_camera_height(in robot frame)+ 0.21658(vertical translation from course to robot)-req.point2.z(height of object)
    '''
    
    #command filter: handles pid range as well as minimum commands
    #if (p < ((diffheight/2)*0.71428571428571))
        #p=

    diffheight = (end_camera_z-req.point2.z)
    py=((-req.point2.x + req.point1.x)*(diffheight-0.04)/8.79167)/100*0.5
    print 'py', py
    #ycomp1=((end_camera_x-0.03442)-(p))
    #print "end_camera_height", end_camera_height
    #print "h*p/c" 
    #print "p", p/100
    #xcomp1=((end_camera_x)-25*((req.point2.x + req.point1.x)/(10000000*(end_camera_height+0.18-req.point2.z))))
    #xcomp1=((end_camera_x)+(1.0/840)*(end_camera_height+0.21658+0.15-req.point2.z)*(req.point2.y + req.point1.y))
    
    ycomp1=end_camera_y+py
    d1= end_camera_z
    print 'd1', d1
    px=((req.point2.y - req.point1.y)*(diffheight-0.04)/8.79167)/100*0.7
    print 'px', px
    print 'd1', d1 
    print 'end_camera_x', end_camera_x
    xcomp1=(((end_camera_x)-px))

    #xcomp1=0.034+((end_camera_x)+((-req.point2.y + req.point1.y)*((end_camera_height-req.point2.z)/8.79167)/50*0.2))
    #ycomp1=((end_camera_y)+(1.0/840)*(end_camera_height+0.21658+0.15-req.point2.z)*(req.point2.x + req.point1.x))


    dist1 = (xcomp1, ycomp1, 0.150) 
    dist2 = Point(*dist1)           #convert from list to Point msg
    #print "dist", dist
    #print "Returning vector components [%s , %s to %s , %s = %s]"%(req.point1.x, req.point1.y, req.point2.x, req.point2.y, (dist))
    
    
    dist = PointStamped()               #make a point stamped to send to the arm ontroller
    dist.header.stamp = rospy.Time.now()
    dist.header.frame_id = '/robot'
    dist.point = dist2
    #dist = (header.frame_id, header.stamp, point)
    print dist
    return dist


def compute_distance_server():
    #rospy.init_node('compute_distance_server')
    s = rospy.Service('compute_distance', ComputeDistance, handle_two_points)
    print "Ready to compute distance."
    #rospy.spin()

if __name__== "__main__":
    

    compute_distance_server()
    rospy.init_node('ieee2015_tf_broadcaster2')#I think this is wrong
    
    #For getting current camera position
    
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/robot','/wrist_joint', rospy.Time(0)) #change to /end_camera frame 
            end_camera_x=trans[0]
            end_camera_y=trans[1]
            end_camera_z = trans[2]
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        #print end_camera_height

        rate.sleep()

    


#Meaningless Change