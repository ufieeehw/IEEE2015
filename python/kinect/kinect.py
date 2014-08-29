
from __future__ import division
import cv2
import freenect
import numpy as np
import cv
import frame_convert
import time
import random
import signal
keep_running = True
last_time = 0

def nothing(x):
    pass
def ensure_odd(x):
    return(x+ (x%2))    

keep_running = True
class kinect_controller:
    
    def __init__(self):         
        print('Press ESC in window to stop')
        self.depth_image = None;
        freenect.runloop(depth=self.do_depth,
                         video=self.do_rgb,
                         body=self.body)
        self.th1, self.th2 = 0,0
    
    def do_depth(self, dev, data, timestamp):
        global keep_running

        depth_image = data
        depth_image_fix = np.uint8(255*(depth_image/2047))
        cv2.imshow('Normalized Depth', depth_image_fix)

        self.th1 = cv2.getTrackbarPos('Th1','Frame')
        self.th2 = cv2.getTrackbarPos('Th2','Frame')

        edges = cv2.Canny(depth_image_fix, self.th1, self.th2, apertureSize=7)
        cv2.imshow('Depth Edges',edges)

        self.depth_image = depth_image
        
        if cv2.waitKey(50) == 27:
            keep_running = False
            cv2.destroyAllWindows()

    def do_rgb(self, dev, data, timestamp):
        global keep_running

        rgb_image = np.uint8(data)

        edges2 = cv2.Canny(rgb_image, self.th1,self.th2, apertureSize=5)

        cv2.imshow('RGB Edges', edges2)
        if cv2.waitKey(50) == 27:
            keep_running = False
            cv2.destroyAllWindows()
            
    def body(self,*args):
        if not keep_running:
            raise freenect.Kill

cv2.namedWindow('Frame')

cv2.createTrackbar('Th1','Frame',10,3000,nothing)
cv2.createTrackbar('Th2','Frame',10,3000,nothing)



k_con = kinect_controller()
