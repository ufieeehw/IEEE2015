

# removes supurluous points
def removeExcess(coordinates, newCoordinates):
    #threshold of 7
    newCoordinates.append(coordinates[0])
    r = 0
    for i in range(0, len(coordinates)):
        if abs(coordinates[i] - newCoordinates[r]) > 20:
            newCoordinates.append(coordinates[i])
            r += 1


def getCoordinates(imageWithStuff):

    img = imageWithStuff

    tempHor = []
    for i in range(0, len(img)):
        if img[i][0][0] == 0 and img[i][0][1] == 0 and img[i][0][2] == 255:
            tempHor.append(i)

    # add verticle lines to an array-- duplicates must also be handled later
    tempVert = []
    for i in range(0, len(img[0])):
        if img[0][i][0] == 0 and img[0][i][1] == 0 and img[0][i][2] == 255:
            tempVert.append(i)

    # create new arrays without the excess points
    horizontal = []
    verticle = []

    removeExcess(tempHor, horizontal)
    removeExcess(tempVert, verticle)

    #create an array with (x,y) coordinates for every corner
    coordinates = []
    for j in range(0, len(horizontal)):
        for i in range(0, len(verticle)):
            coordinates.append([verticle[i], horizontal[j]])

    return coordinates
