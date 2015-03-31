import math


def get_mid_quarter_points(points, ma, MA):
    minAxisMid = []
    majAxisMid = []
    quarterMin1 = []
    quarterMin2 = []
    quarterMaj1 = []
    quarterMaj2 = []

    for i in range(0, 4):
        for j in range(0, 4):
            tempnum = math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1])
            if tempnum < ma + 10 and tempnum > ma - 10:
                #calculate midpoint between that stuff
                midpoint = ((points[i][0] + points[j][0])/2, (points[i][1] + points[j][1])/2)
                minAxisMid.append(midpoint)
                temppoint1 = ((midpoint[0] + points[i][0])/2, (midpoint[1] + points[i][1])/2)
                quarterMin1.append(temppoint1)
                temppoint2 = ((midpoint[0] + points[j][0])/2, (midpoint[1] + points[j][1])/2)
                quarterMin2.append(temppoint2)

            elif tempnum < MA + 10 and tempnum > MA - 10:
                midpoint = ((points[i][0] + points[j][0])/2, (points[i][1] + points[j][1])/2)
                majAxisMid.append(midpoint)
                temppoint1 = ((midpoint[0] + points[i][0])/2, (midpoint[1] + points[i][1])/2)
                quarterMaj1.append(temppoint1)
                tempopint2 = ((midpoint[0] + points[j][0])/2, (midpoint[1] + points[j][1])/2)
                quarterMaj2.append(temppoint2)

    list(minAxisMid)
    list(majAxisMid)

    minAxisMid.sort()
    majAxisMid.sort()

    list(quarterMin1)
    list(quarterMin2)
    list(quarterMaj1)
    list(quarterMaj2)

    quarterMin1.sort()
    quarterMin2.sort()
    quarterMaj1.sort()
    quarterMaj2.sort()

    #print 'should be sorted'
    #print majAxisMid
    #print minAxisMid
    #print quarterMaj1
    #print quarterMaj2
    #print quarterMin1
    #print quarterMin2

    ###Aka corner points in which to decide line for major and minor axis lines
    #p1CentMinor = minAxis[0]
    #p2CentMinor = minAxis[2]a

    #p1CentMajor = majAxis[0]
    #p2CentMajor = majAxis[2]

    return minAxisMid, majAxisMid, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2
