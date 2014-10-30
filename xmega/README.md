XMega Communication Protocol
============================

This defines the serial interface between the XMega and the robot's main computer

# General API Information
Everything in the robot will run off of messages sent by the computer or hardware/timing interrupts.
Message topics are defined in types.h.
* Message topics are broken into 4 groups, based on the data requirements of each message (0,1,2, or N byte data packages) -The last 16 values in each group are reserved for error codes (can be more if needed).
  * Error Codes are limited to one per system, if you need more, use the message's data field
* To subscribe to a message topic, create a function and add it to the list in table.c with the same value as the topic
  * Your function must accept only a message as an argument, and must return an integer error code (OK is the norm)
  * Do not free the data buffer in the message, parent function will handle that

To send a message, call queue_push(Message *m, int direction).
* Direction should either be IN_QUEUE (for internally handled messages) or OUT_QUEUE(for computer bound messages)
* Function will create a new copy of the message, but not the data (Message->data). 
* Use get_msg(type, size) to allocate messages

If you need to do Initializations (ie for interrupts)
* Put a function call to your initialization method in main.c's init() function.

#How to Install Driver Code for Dummies
1. Write your functions
2. Create a .h file with the function prototypes
3. Register a type in types.h (talk to Josh)
4. Include part 2 in table.c
5. Add your function to the array (same number as your type)
6. Do you have initializations? If not, go to 9.
7. Include part 2 in main.c
8. Add your init function call to init() in the marked section.
9. Congratulations, you've successfully installed your driver.

Feel free to ask me (Josh) if you have any questions.




