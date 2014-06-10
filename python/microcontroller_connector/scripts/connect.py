#ROS code to talk to the microcontroller, inspired by the code for 2014
import rospy
import serial

from microcontroller_connector.msg import packet
from somethingToPush import push #this is just a temporary placeholder example

pub = rospy.Publisher("micro_rx", packet) #we will publish all received packets here
rospy.init_node("microcontroller_connector") #our node
com = serial.Serial() #our serial communication
com.baudrate = 9600 #the baudrate of our communications, 09JUN2014 should probably be higher
com.timeout = 10 #10 seconds timeout
com.port = 0 #09JUN2014 how to detect where the microcontroller is? Or will it be static?

def testReceive(come): #just to test receiving bytes
  rospy.log("Testing receiving from microcontroller")
  while True:
    rospy.log("%s", com.read(1))

def receive(com):
  rospy.log("Receiving from microcontroller")
  data = '' #where the received data will be stored
  startCharsCount = 0 #we have to receive ^^^ to know the packet has started
  dataLength = 0 #immediately following ^^^ will be a character representing how many bytes of data we will receive
  com.open() #do I really need to do this?
  pac = packet()

  while True:
    if start_chars_count < 3: #waiting for start bits
    #TODO: Implement timeout
      if com.read(1) == '^':
        startCharsCount += 1
        rospy.log("Received start char %i of 3", startCharsCount)
    else: #we're getting a packet! I'm preserving the packet structure from 2014, it seemed solid
      pac.error = 0
      dataLength = ord(com.read(1)) #nifty trick I totally stole from Forrest, turns string character into it's unicode representation
      pac.msgLength = dataLength

      rospy.log("Receiving %i bytes from microcontroller", dataLength)
      data = com.read(dataLength) #receive dataLength bytes 

      pac.timeStamp = rospy.Time.now().nsecs

      pac.msgType = data[0]
      rospy.log("we received a packet of type %s", pac.msgType)

      pac.msgData = data[1:]
      rospy.log("Pac body: %s", pac.msgData)

      if pac.msgData.length != pac.msgLength:
        pac.error = 1
        rospy.log("Error reading packet")

      com.close() #Do I really need to do this?
      return pac
