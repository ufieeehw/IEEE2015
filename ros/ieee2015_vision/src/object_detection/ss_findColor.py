import cv2


def findColor(brightX, brightY, blueX, blueY, greenX, greenY, minXPointL, maxXPointL, minYPointL, maxYPointL, minXPointR, maxXPointR, minYPointR, maxYPointR):
    averageRow = (greenX + blueX)/2

    if(brightX > averageRow and brightY > greenY and brightY < blueY):
        return 1 #  we have a yellow button lighting up
    elif(brightX < averageRow and brightY > greenY and brightY < blueY):
        return 3  #  we have a red button lighting up
    elif(brightX < maxXPointL and brightX > minXPointL and brightY < maxYPointL and brightY > minYPointL):
        return 2
    elif(brightX < maxXPointR and brightX > minXPointR and brightY < maxYPointR and brightY > minYPointR):
        return 4
    else:
        return -1