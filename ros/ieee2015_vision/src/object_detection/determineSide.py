def determineSide(point1Major, point2Major, point1Minor, point2Minor, pointOfInterest, angle):
	minorVal = 0
	majorVal = 0
	if (((point2Minor[0] - point1Minor[0]) * (pointOfInterest[1] - point1Minor[1])) - ((point2Minor[1] - point1Minor[1])*(pointOfInterest[0] - point1Minor[0]))) > 0:
		minorVal = 1

	if (((point2Major[0] - point1Major[0]) * (pointOfInterest[1] - point1Major[1])) - ((point2Major[1] - point1Major[1])*(pointOfInterest[0] - point1Major[0]))) > 0:
		majorVal = 1

	return majorVal, minorVal