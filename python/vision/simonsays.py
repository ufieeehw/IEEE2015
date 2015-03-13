import cv2
import numpy as np

def getSSMoves():
    
lower_blue_off = np.array([100, 60, 45])
upper_blue_off = np.array([110, 255, 255])
    
lower_blue_on = np.array([80, 175, 175])
upper_blue_on = np.array([100, 255, 255])
    
lower_green_off = np.array([65, 45, 45])
upper_green_off = np.array([80, 255, 255])
    
lower_green_on = np.array([80, 175, 175])
upper_green_on = np.array([100, 255, 255])
    
lower_red_off = np.array([0, 100, 100])
upper_red_off = np.array([20, 255, 255])
    
lower_red_on = np.array([165, 175, 175])
upper_red_on = np.array([180, 255, 255])
    
lower_yellow_off = np.array([30, 70, 50])
upper_yellow_off = np.array([50, 255, 255])

lower_yellow_on = np.array([25, 175, 175])
upper_yellow_on = np.array([35, 255, 255])


class SimonSays:
          
    def __init__(self):
        gameStarted = False
        sequenceRestarted = True
        currentGivenSequenceIndex = 0
        colorSequence = []
        blueLit = False
        greenLit = False
        redLit = False
        yellowLit = False
        
        
        
        self.blueRegion = np.ndarray([1,1,1])
        self.greenRegion = np.ndarray([1,1,1])
        self.redRegion = np.ndarray([1,1,1])
        self.yellowRegion = np.ndarray([1,1,1])
        
        self.blueMask = np.ndarray([1,1,1])
        self.greenMask = np.ndarray([1,1,1])
        self.redMask = np.ndarray([1,1,1])
        self.yellowMask = np.ndarray([1,1,1])
        
        
        self.blueRegionG = np.ndarray([1,1,1])
        self.greenRegionG = np.ndarray([1,1,1])
        self.redRegionG = np.ndarray([1,1,1])
        self.yellowRegionG = np.ndarray([1,1,1])
        
        
    def detectColor(self, imgSrc):
        
        """
        first call should end after first color is detected, eg sequenceRestarted = True and len(colorSequence) = 0
        
        if len(colorSequence) != 0 && sequenceRestarted = True 
??      bother comparing detected color to colorSequence?
        otherwise detect color, set colorLit to True, once that changes, currentSequenceIndex++
        once currentSequenceIndex == len(colorSequence) call addToSequence with the latest color
        
??      indicate in some way that the color went off so that we can start entering the sequence
        
        
        
        
        take input image, compare regions with unlit color, if different, colorLit = true (error check if multiple colorLit's are true).
        If only one color is lit, call addToSequence with that color input, set colorLit to false.
        
        
        """
        
        
        
        
        #Detect color from input image, adds to sequence array
        inputImage = cv2.imread(imgSrc, cv2.IMREAD_COLOR)

        # Used for morphology 
        #temp = np.array(0)
        #kernel = np.ones((150,150),np.uint8)
        
        grayImage = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
        # Converts input to HSV format, sets up color ranges
        hsvImage = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
        
        
        
        # Masks out unlit blue region
        blueInput = cv2.bitwise_and(inputImage, inputImage, mask= self.blueMask)

        
        
        # Masks out unlit green region
        greenInput = cv2.bitwise_and(inputImage, inputImage, mask= self.greenMask)
        
        # Masks out unlit red region
        redInput = cv2.bitwise_and(inputImage, inputImage, mask= self.redMask)
        
        # Masks out unlit yellow region
        yellowInput = cv2.bitwise_and(inputImage, inputImage, mask= self.yellowMask)
        
        cv2.namedWindow('img1', cv2.WINDOW_NORMAL)
        cv2.imshow('img1', blueInput)
        
        cv2.namedWindow('img2', cv2.WINDOW_NORMAL)
        cv2.imshow('img2', greenInput)
        
        cv2.namedWindow('img3', cv2.WINDOW_NORMAL)
        cv2.imshow('img3', redInput)
        
        cv2.namedWindow('img4', cv2.WINDOW_NORMAL)
        cv2.imshow('img4', yellowInput)
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        


    def addToSequence(self):
        """
??      takes color input (int? char?) and adds it to the end of a sequence.
        
        """
        
        
        
        
        
        
    def returnColorLocation(self):
        #given color from sequence array, determines location and returns it
        """
??      reads colorSequence from 0 to end, for each item in the sequence call helper method getColorSequence and ?Print? color location
        
        
        
        
        
        i = 0
??      while(colorSequence[i] != null)
            print getColorLocation(colorSequence[i])
            i++
        
        
        
        return something to indicate the sequence is done
        """
        
        
        
        
    def getColorLocation(self):
            
        """
        takes image input, returns location of color
            
        """
            
            
    def run(self):
        self.getRegions('/home/will/Desktop/Simon Images/IMG_4304.JPG')  
        self.detectColor('/home/will/Desktop/Simon Images/IMG_4314.JPG') 
 
 
    def getRegions(self, imgSrc):
        #Detect color from input image, adds to sequence array
        inputImage = cv2.imread(imgSrc, cv2.IMREAD_COLOR)

        # Used for morphology 
        #temp = np.array(0)
        #kernel = np.ones((150,150),np.uint8)
        
        
        # Converts input to HSV format, sets up color ranges
        hsvImage = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)

        
        # Masks out unlit blue region
        self.blueMask = cv2.inRange(hsvImage, lower_blue_off, upper_blue_off)
        self.blueRegion = cv2.bitwise_and(inputImage, inputImage, mask= self.blueMask)
        self.blueRegionG = cv2.cvtColor(self.blueRegion, cv2.COLOR_BGR2GRAY)
        
        print type(self.blueMask)
        print type(self.blueRegion)
        print self.blueRegion.shape
        
        
        # Masks out unlit green region
        self.greenMask = cv2.inRange(hsvImage, lower_green_off, upper_green_off)
        self.greenRegion = cv2.bitwise_and(inputImage, inputImage, mask= self.greenMask)
        self.greenRegionG = cv2.cvtColor(self.greenRegion, cv2.COLOR_BGR2GRAY)
        
        # Masks out unlit red region
        self.redMask = cv2.inRange(hsvImage, lower_red_off, upper_red_off)
        self.redRegion = cv2.bitwise_and(inputImage, inputImage, mask= self.redMask)
        self.redRegionG = cv2.cvtColor(self.redRegion, cv2.COLOR_BGR2GRAY)
        
        # Masks out unlit yellow region
        self.yellowMask = cv2.inRange(hsvImage, lower_yellow_off, upper_yellow_off)
        self.yellowRegion = cv2.bitwise_and(inputImage, inputImage, mask= self.yellowMask)
        self.yellowRegionG = cv2.cvtColor(self.yellowRegion, cv2.COLOR_BGR2GRAY)
        
        cv2.namedWindow('img1', cv2.WINDOW_NORMAL)
        cv2.imshow('img1', self.blueRegionG)
        
        cv2.namedWindow('img2', cv2.WINDOW_NORMAL)
        cv2.imshow('img2', self.greenRegionG)
        
        cv2.namedWindow('img3', cv2.WINDOW_NORMAL)
        cv2.imshow('img3', self.redRegionG)
        
        cv2.namedWindow('img4', cv2.WINDOW_NORMAL)
        cv2.imshow('img4', self.yellowRegionG)
        
        
        
        cv2.namedWindow('img5', cv2.WINDOW_NORMAL)
        cv2.imshow('img5', self.blueMask)
        
        cv2.namedWindow('img6', cv2.WINDOW_NORMAL)
        cv2.imshow('img6', self.greenMask)
        
        cv2.namedWindow('img7', cv2.WINDOW_NORMAL)
        cv2.imshow('img7', self.redMask)
        
        cv2.namedWindow('img8', cv2.WINDOW_NORMAL)
        cv2.imshow('img8', self.yellowMask)
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
 
 
 
 
 
SS = SimonSays()
SS.run()
"""
 
 Questions:
 
 How should button location be returned? print? 
 
 Reliability of region before and after pressing the button? How much will pressing the button move the device?
 
 Timing between presses?
 
 
 

 
 
 
 retry if failure? I think only sound can indicate that. 
 
 """      