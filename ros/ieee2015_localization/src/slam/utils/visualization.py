def _draw_features(image, kp, bird_corners, intype):
    if intype == "points":
        # Turn this image into color so we see green dots
        color_img = cv2.cvtColor(image , cv2.COLOR_GRAY2RGB)
        # Plot the keypoints
        for i in range(0,len(kp)):
            # Extract coordinates of these keypoints
            feature1 = kp[i].pt 
            # Offset feature2
            feature1 = tuple([int(feature1[0]), int(feature1[1])]) 
            # Put dots on these features
            cv2.circle(color_img, feature1, 2, (0, 0, 255), -1) 

        if (bird_corners != None):
            cv2.line(color_img, tuple([int(bird_corners[0][0]), int(bird_corners[1][0])]),
                                tuple([int(bird_corners[0][1]), int(bird_corners[1][1])]),
                                (255, 0, 0))
            cv2.line(color_img, tuple([int(bird_corners[0][1]), int(bird_corners[1][1])]),
                                tuple([int(bird_corners[0][2]), int(bird_corners[1][2])]),
                                (255, 0, 0))
            cv2.line(color_img, tuple([int(bird_corners[0][2]), int(bird_corners[1][2])]),
                                tuple([int(bird_corners[0][3]), int(bird_corners[1][3])]),
                                (255, 0, 0))
            cv2.line(color_img, tuple([int(bird_corners[0][3]), int(bird_corners[1][3])]),
                                tuple([int(bird_corners[0][0]), int(bird_corners[1][0])]),
                                (255, 0, 0))
        print 'drawing features 1'
        cv2.imshow('full', color_img) 
        cv2.waitKey(0)

    elif intype == "array":
        # Turn this image into color so we see green dots
        color_img = cv2.cvtColor(image , cv2.COLOR_GRAY2RGB)
        # Plot the keypoints
        for i in range(0,len(kp)):
            # Offset feature2
            feature1 = tuple([int(kp[i][0]), int(kp[i][1])]) 
            # Put dots on these features
            cv2.circle(color_img, feature1, 2, (0,0,255), -1) 

        print 'drawing features 2'
        cv2.imshow('full', color_img) 
        cv2.waitKey()