
from __future__ import division
import cv2
import freenect
import numpy as np
import cv
import frame_convert
import time
import random
import signal
#import frame_convert

keep_running = True
last_time = 0

cv.NamedWindow('Depth')
cv.NamedWindow('RGB')
keep_running = True
class kinect_controller:
    
    def __init__(self):         
        print('Press ESC in window to stop')
        self.depth_image = None;
        freenect.runloop(depth=self.do_depth,
                         video=self.do_rgb,
                         body=self.body)
    
    def do_depth(self, dev, data, timestamp):
        global keep_running
        #cv.ShowImage('Depth', frame_convert.pretty_depth_cv(data))
        depth_image = data
        depth_image_fix = (depth_image - np.min(depth_image))/np.max(depth_image)
        cv2.imshow('Hello',255*np.uint8(depth_image_fix > 0.1))
        
        self.depth_image = depth_image

        if cv.WaitKey(10) == 27:
            keep_running = False

    def do_rgb(self, dev, data, timestamp):
        global keep_running
        
        if self.depth_image is not None:
            cv2.imshow('RGB*D', (self.depth_image < 800)*data[:,:,1])
        else:
            print "None"
        cv.ShowImage('RGB', frame_convert.video_cv(data))
        
        if cv.WaitKey(10) == 27:
            keep_running = False

    def body(self,*args):
        if not keep_running:
            raise freenect.Kill

k_con = kinect_controller()
