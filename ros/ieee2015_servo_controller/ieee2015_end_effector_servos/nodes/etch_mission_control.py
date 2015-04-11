import client_close_small
import client_open_small
import client_down
import client_open_large
import client_close_large
import client_up
from ieee2015_end_effector_servos.msg import high_level
import etch_publisher
import rospy

calibrate = 0


if __name__ == "__main__":
    
    rospy.init_node('etch_mission', anonymous=True)

    pub = rospy.Publisher('end_efffector_angles_sub', high_level, queue_size=1)
    pub.publish(calibrate,calibrate, None)

    temp = client_close_small.main()
    while temp != None:
        None
    temp = client_close_small.main()
    while temp != None:
        None
    temp = client_close_small.main()
    while temp != None:
        None
    temp = client_close_large.main()
    while temp != None:
        None
    temp = client_down.main()
    while temp != None:
        None

    temp = etch_publisher.main()
    while temp == None:
        None

    temp = client_up.main()
    while temp != None:
        None

    temp = client_open_large.main()
    while temp != None:
        None

    temp = client_open_small.main()
    while temp != None:
        None

