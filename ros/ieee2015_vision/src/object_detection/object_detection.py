import simon_says
import cardDetection
import rubix
import etchasketch

'''From what team has said each method will return points of interest to mission control,
question is, how do we know which object we are dealing with? we can go by color perhaps?'''
#xCoordToPush = 0
#yCoordToPush = 0


def object_detection(img):
    #chain of if statements to determine what operations to perform on which objects
    if(OBJECT IS A RUBIX CUBE):
        rubix.findRubix(img)
    
    elif(OBJECT IS SIMON SAYS):
        #don't know how to set up timing with this one since has to be dynamic
        #EXTREME pseudocode to get point across of when everything need to be done
        #get standard keeps being called to give reference to where the points are relative
        #to the toy, in case the toy moves when pushed etc
        while(we still havent reached time limit)
            if(no image has been take yet or button has been pushed, or color lights up)
                simon_says.setStandard(img)

            #since it has to keep track the points returned are in a list
            buttonCordList = coorsimon_says.playSimonSays(img)
    
    elif(IMAGE IS ETCHA SKETCH):
        #each return value will actually have two values in them, since two circles
        xCoordsCircles, yCoordsCircles = etchasketch.etchasketch(img)

    elif(OBJECT IS CARDS):
        xCoordToPush, yCoordToPush = cardDetection.getCardLoc(img)

    else:
        continue #tb filled out


