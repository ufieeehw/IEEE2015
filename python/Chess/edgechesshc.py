import cv2
import numpy as np

picThresh = 126
lineThresh = 145
density = 0.1


def addEdges(filename):
    image = cv2.imread(filename)

    #Remove edge noise
    image_blurred = cv2.GaussianBlur(image, (3, 3), 0)

    #Convert to gray
    image_gray = cv2.cvtColor(image_blurred, cv2.COLOR_RGB2GRAY)

    #Grad in x and y used to find sudden changes in the image, such as an edge
    grad_x = cv2.Scharr(image_gray, cv2.CV_32F, 1, 0)

    grad_y = cv2.Scharr(image_gray, cv2.CV_32F, 0, 1)

    #Overall grad, pythagorum
    grad_x = pow(grad_x, 2, grad_x)
    grad_y = pow(grad_y, 2, grad_y)

    grad = grad_x + grad_y
    grad = np.sqrt(grad)

    #Convert to 8 bit depth
    edges = np.uint8(grad)
    ret, edges_thresholded = cv2.threshold(edges, picThresh, 255, cv2.THRESH_TOZERO)

    #Lines
    edges_color = cv2.cvtColor(edges_thresholded, cv2.COLOR_GRAY2RGB)
    edges_line = edges_color.copy()

    lines = cv2.HoughLines(edges_thresholded, density, np.pi / 180, lineThresh)
    for rho, theta in lines[0]:
        if((-.005 < theta and theta < .005) or (1.55 < theta and theta < 1.59)):
            a = np.cos(theta)
            b = np.sin(theta)

            x0 = a * rho
            y0 = b * rho

            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))

            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))

            cv2.line(edges_line, (x1, y1), (x2, y2), (0, 0, 255), 2)

    return edges_line
