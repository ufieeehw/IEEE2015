def findColor(brightX, brightY, blueX, blueY, greenX, greenY, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR):
    averageRow = (greenX + blueX)/2

    print 'this is average row'
    print averageRow

    print 'this is bright center x'
    print brightX
    print 'this is bright center y'
    print brightY

    print 'this is greenY'
    print greenY
    print 'this is greenX'
    print greenX

    print 'this is blueY'
    print blueY
    print 'this is blueX'
    print blueX

    #  means that centerpoint of the bright spot is below the average row of green and blue
    if(brightX > averageRow and brightY > greenY and brightY < blueY):
        return 1  # we have a yellow button lighting up
    #  means that the centerpoint of the bright spot is above the average row of green and blue
    elif(brightX < averageRow and brightY > greenY and brightY < blueY):
        return 3  # we have a red button lighting up
    #  means that the centerpoint of the bright spot is between the boundaries of the green button
    elif(brightX < maxXL and brightX > minXL and brightY < maxYL and brightY > minYL):
        return 2  # we have a green button
    #  mean the centerpoint of the bright spot is between the boundaries of the blue button
    elif(brightX < maxXR and brightX > minXR and brightY < maxYR and brightY > minYR):
        return 4  # we have a blue button
    else:
        return -1