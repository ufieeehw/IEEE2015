import numpy as np
import cv2
import glob


def chessCalibrate():  # need to change parameters
    # termination criteria
    #aka some weird ass opencv stuff and specific operators
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    #makes 8x8 object grid with 3 elements per object
    objp = np.zeros((8 * 8, 3), np.float32)
    objp[:, :2] = np.mgrid[0: 8, 0: 8].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    #all of our test images, tutorial recommended 10 at minimum
    testimages = glob.glob('./Chessboard/*.jpg') 
    

    gray = []
    for current_img in testimages:
        img = cv2.imread(current_img)
        img = cv2.resize(img, (576, 384))
        #cv2.imshow('file pictures', img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (6, 6), None)
        print ret
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)  # figure out what 11, 11 is and -1 and -1
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (8, 8), corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(5000)

    cv2.destroyAllWindows()


    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


#def undistort(mtx, dist, objpoints, imgpoints, rvecs, tvecs, img):
    img = cv2.imread('cb1.jpg')
    #read in an image of choice
    #usefule for the reading in and segmenting of board to make hough lines easier
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    cv2.imwrite('calibresult.png', dst)

    # undistort
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    # crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    cv2.imwrite('calibresult.png', dst)

    mean_error = 0
    for i in xrange(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print "total error: ", mean_error / len(objpoints)

chessCalibrate()