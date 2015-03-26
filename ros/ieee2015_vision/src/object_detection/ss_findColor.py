import cv2

def findColor(img, brightCol, brightRow, blueCol, blueRow, greenCol, greenRow, minColGreen, maxColGreen, minRowGreen, maxRowGreen, minColBlue, maxColBlue, minRowBlue, maxRowBlue):
    #img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
    print 'this is mean row green'
    print greenRow
    print 'this is mean col green'
    print greenCol
    print 'this is mean row blue'
    print blueRow
    print 'this is mean col blue'
    print blueCol

    averageRow = int((greenCol + blueCol)/2)
    averageCol = int((greenRow + blueRow)/2)


    cv2.circle(img,(brightCol, brightRow),2,(0,0,255),30)
    img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('findColor', img)
    cv2.waitKey(0)

    print 'this is average row'
    print averageRow

    print 'this is average col'
    print averageCol

    print 'this is bright center x'
    print brightCol
    print 'this is bright center y'
    print brightRow

    print 'this is maxColBlue'
    print maxColBlue

    print 'this is minColBlue'
    print minColBlue

    print 'this is maxRowBlue'
    print maxRowBlue

    print 'this is minRowBlue'
    print minRowBlue

    print 'this is greenRow'
    print greenRow
    print 'this is greenCol'
    print greenCol

    print 'this is blueRow'
    print blueRow
    print 'this is blueCol'
    print blueCol

    #  means that centerpoint of the bright spot is below the average row of green and blue
    if(brightRow > averageRow and brightCol > greenCol and brightCol < blueCol):
        return 1  # we have a yellow button lighting up
    #  means that the centerpoint of the bright spot is above the average row of green and blue
    elif(brightRow < averageRow and brightCol > greenCol and brightCol < blueCol):
        return 3  # we have a red button lighting up
    #  means that the centerpoint of the bright spot is between the boundaries of the green button
    elif(brightCol < maxColGreen and brightCol > minColGreen and brightRow < maxRowGreen and brightRow > minRowGreen):
        return 2  # we have a green button
    #  mean the centerpoint of the bright spot is between the boundaries of the blue button
    elif(brightCol < maxColBlue and brightCol > minColBlue and brightRow < maxRowBlue and brightRow > minRowBlue):
        return 4  # we have a blue button
    else:
        return -1