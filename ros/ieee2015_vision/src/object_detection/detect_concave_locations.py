import cv2
import numpy as np

def detect_concave_locations(contour, img):
fects =
hull = cv2.convexHull(contour, returnPoints = False)
defects = cv2.convexityDefects(contour, hull)

for i in range(defects.shape[0]):
    far = tuple(cnt[f][0])
    cv2.line(img,start,end,[0,255,0],2)
    cv2.circle(img,far,5,[0,0,255],-1)

cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()