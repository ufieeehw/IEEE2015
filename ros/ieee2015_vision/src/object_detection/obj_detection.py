import cv2
import numpy as np
from . import simon_says
from . import rubix
from . import etchasketch
from . import cardDetection

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

