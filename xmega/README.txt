**Work in progress**

--- Instructions for using API ---
Everything in the robot will run off of messages sent by the computer or hardware/timing interrupts.
Message topics are defined in types.h.
 -Message topics are broken into 4 groups, based on the data requirements of each message (0,1,2, or N byte data packages)
 -The last 16 values in each group are reserved for error codes (can be more if needed).
  --Error Codes are limited to one per system, if you need more, use the message's data field
 -To subscribe to a message topic, create a function and add it to the list in table.c with the same value as the topic
  --Your function must accept only a message pointer as an argument, and must return an integer error code (OK is the norm)

*ALWAYS* free the memory when done reading a message, and make sure to use calloc() to create a new message.

To send a message, call queue_push(Message *m, int direction).
 -Dierction should either be IN_QUEUE (for internally handled messages) or OUT_QUEUE(for computer bound messages)

If you need to do Initializations (ie for interrupts)
 -Put a function call to your initialization method in main.c's init() function.

Feel free to ask me (Josh) if you have any questions.




