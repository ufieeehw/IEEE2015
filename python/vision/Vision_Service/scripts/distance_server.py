#!/usr/bin/env python
import rospy
#takes two points and a height of an object
#looks up the transform from world(/course) to the downward facing camera(/end_camera)
#computes the real world distance between the two points by applying a camera specific constant(pixles/meters) derived from experimenting with the camera 
def handle_two_points(req):
    pixeldist = math.hypot(req.point1.x - req.point2.x, req.point1.y - req.point2.y)
    dist = pixeldist*(end_camera_height-req.h)*(1/84000)
    print "Returning distance [%s , %s to %s , %s = %s]"%(req.point1.x, req.point1.y, req.point2.x, req.point2.y, (dist))
    return ComputeDistanceResponse(dist)


def compute_distance_server():
    rospy.init_node('compute_distance_server')
    s = rospy.Service('compute_distance', ComputeDistance, handle_two_points)
    print "Ready to compute distance."
    rospy.spin()

if __name__== "__main__":
    rospy.init_node('ieee2015_tf_broadcaster')
    listener = tf.TransformListener()
    (trans,rot) = listener.lookupTransform('/end_camera', '/course', rospy.Time(0))
    end_camera_height=trans[2]# also try trans[0]
    compute_distance_server()
