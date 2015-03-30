import cv2
import getMidPoints
import determineSide
import ss_getLitUpButton

def getColor(img):
	x, y, p1CentMajor, p2CentMajor, p1CentMinor, p2CentMinor, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2 = getMidPoints.getAxisPoints(img)

	pointOfInterest = (550, 100)

	majorVal, minorVal = determineSide.determineSide(p1CentMajor, p2CentMajor, p1CentMinor, p2CentMinor, pointOfInterest)
	print minorVal
	print majorVal

	
	
img = cv2.imread('Images/ssR.jpg')
img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
getColor(img)