import cv2
import numpy as np
from . import ss_run_simon_says
from . import rubix
from . import etchaSketch_detect
from . import detect_card

'''this is the object detection driver
each of the object detection methods will be called from here.
simon says being the most complicated of them all has its own separate
driver in the file called simon_says.py. Card Detection should be taken care of
in the cardDetection.py file. Rubix cube detection is called from rubix.py. 
And finally, Etch a sketch detection is called from etchasketch.py. 
All methods should return points of interest in pixel coordinates as well
as an angle of orientation. 

TO DO:
determine how to know which object we are at
Look over jake orientation method used in slam
solidify detection of circles of etcha sketch
rosify everything
'''

def detectObjects(img):
    #chain of if statements to determine what operations to perform on which objects
    if(OBJECT IS A RUBIX CUBE):
        rubix.find_rubix(img)
    
    elif(OBJECT IS SIMON SAYS):
        #don't know how to set up timing with this one since has to be dynamic
        #EXTREME pseudocode to get point across of when everything need to be done
        #get standard keeps being called to give reference to where the points are relative
        #to the toy, in case the toy moves when pushed etc
        while(we still havent reached time limit)
            if(no image has been take yet or button has been pushed, or color lights up)
                ss_run_simon_says.get_std_fields(img)
            else: #while we are still running everything 
                ss_run_simon_says.add_color(img)  #figures out what colors are being pushed
    
    elif(IMAGE IS ETCHA SKETCH):
        #each return value will actually have two values in them, since two circles
        cx_coord, cy_coord, angle = etchaSketch_detect.etchaSketch_detect(img)

    elif(OBJECT IS CARDS):
        center_x, center_y, angle = detect_card.find_card(img, height)

    else:
        continue #tb filled out




