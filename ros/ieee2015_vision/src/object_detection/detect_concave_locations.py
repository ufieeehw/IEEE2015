import cv2
import numpy as np

def detect_concave_locations(contour, img, draw):
    #get convext hull
    hull = cv2.convexHull(contour, returnPoints = False)
    #get defects in convexity
    #each row of defects containts the following:
    #(startPoint_of_defect, endPoint_of_defect, farthest point from hull, distance from hull)
    defects = cv2.convexityDefects(contour, hull)

    farPoints = []
    for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]
        far = tuple(contour[f][0])
        print 'this is distance'
        print d
        if d > 8000:
            farPoints.append(far)
            if draw is True:
                #cv2.line(img, start, end, [0, 255, 0], 2)
                cv2.circle(img, far, 5, [0, 0, 255], -1)
        
    if draw is True:
        cv2.imshow('img', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return farPoints
