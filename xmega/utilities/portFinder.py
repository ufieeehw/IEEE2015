#This portFinder must be in same directory as other codes you wish to look in
import os
import sys
import inspect
#find path
_path = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0]))
_path = _path[:_path.rfind("/utilities")]
if _path not in sys.path:
    sys.path.insert(0, _path)

#import files from current directory and add it to files    
files = []
for _file in os.listdir(_path):
    if _file.endswith(".c"):
        files.append(_file)

#add what is in the imported .c file to fileTxt Dict
fileTxt = {}
for eachfile in files:
	with open ("../" + eachfile, "r") as myfile:
		fileTxt[eachfile] = myfile.readlines()
myfile.close()

#remove comments from imported files
for eachfile in fileTxt:
	starComment = False
	for eachLine in fileTxt[eachfile]:
		if "//" in eachLine:
			index = eachLine.rfind("//")
			temp = eachLine[:index]
			eachLineIndex = fileTxt[eachfile].index(eachLine)
			fileTxt[eachfile].pop(eachLineIndex)
			fileTxt[eachfile].insert(eachLineIndex,temp)
		if "/*" in eachLine:
			index = eachLine.rfind("/*")
			temp = eachLine[:index]
			eachLineIndex = fileTxt[eachfile].index(eachLine)
			fileTxt[eachfile].pop(eachLineIndex)
			fileTxt[eachfile].insert(eachLineIndex,temp)
		if " * " in eachLine[0:3]:
			index = eachLine.rfind(" * ")
			temp = eachLine[:index]
			eachLineIndex = fileTxt[eachfile].index(eachLine)
			fileTxt[eachfile].pop(eachLineIndex)
			fileTxt[eachfile].insert(eachLineIndex,temp)

#this is for testing purposes only			
"""
for eachfile in fileTxt:
	for eachLine in fileTxt[eachfile]:
		print eachLine		
"""

#Check for all ports and add all ports to portList
portList = dict.fromkeys(files,[])
for eachfile in fileTxt:
	templist = []
	for eachLine in fileTxt[eachfile]:
		if "PORT" in eachLine:
			index = eachLine.rfind("PORT")
			temp = eachLine[index:index+5]
			letter = eachLine[index+4:index+5]
			if ord(letter) >= 65 and ord(letter) <= 90:
				templist.append(temp)
	portList[str(eachfile)] = templist


#remove keys with no ports
tempPortList = []
for eachkey in portList:
	if not len(portList[eachkey]):
		tempPortList.append(eachkey)
for eachkey in tempPortList:
	del portList[eachkey]


#get rid of duplicate ports
for eachfile in portList:
	templist = []
	for value in portList[eachfile]:
		if value not in templist:
			templist.append(value)
	portList[str(eachfile)] = templist


#make list of pins
pinList = dict.fromkeys(portList)
for eachfile in pinList:
	pinList[eachfile] = dict.fromkeys(portList[eachfile])
	for eachport in pinList[eachfile]:
		temp = eachport + "."
		temp2 = eachport + "_"
		pinFound = False
		tempList = []
		for eachLine in fileTxt[eachfile]:
			if temp in eachLine:
				pinFound = True
				index1 = eachLine.rfind(eachport + ".")
				index2 = eachLine.rfind(";")
				tempList.append(eachLine[index1:index2])
			if temp2 in eachLine:
				pinFound = True
				index1 = eachLine.rfind(eachport + "_")
				index2 = eachLine.rfind(";")
				pin = eachLine[index1:index2]
				if "=" in pin:
					tempList.append(eachLine[index1:index2])
		if not pinFound:
			tempList.append("No pins found")
		pinList[eachfile][eachport] = tempList

for eachfile in pinList:
	print eachfile + " uses " + " ".join(pinList[eachfile])
	for eachport in pinList[eachfile]:
		print "    " + eachport + " uses "
		for eachpin in pinList[eachfile][eachport]:
			print "        " + eachpin
	print
