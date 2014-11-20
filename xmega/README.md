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

# How to Install Driver Code for Dummies
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

# Messaging Protocol Explanation
Messages come in three "tokens"
1. Type 
2. Length (Only matters for N-byte messages)
3. Data

Type Byte | (Length Byte if it's an N-Byte Message) | Data Bytes


## Type
Types are defined in types.h. It is a single byte that defines what sort of message is being sent, what length to expect, and whether or not it represents an error. 

The first two bits in the type represent the length. 

| "First two bits" |"Message Length"                                                  |
| ------------- |:-------------------------------------------------------------------:|
| 00            | Empty message - just  type, no data (also called a polling message) |
| 01            | 1 byte message                                                      |
| 10            | 2 byte message                                                      |
| 11            | N-Byte message (The byte that follows type will be length)          |



If both the 3rd and 4th bits are 1, then the message is recognized as an error type

The rest of the type encode what the message, -- better explained, the last 6 bits are the identifier.

Examples of types might be "wheel 1 speed" or "wheel 2 motor fault" or "IMU data message"

## Length

This ONLY matters for N-Byte types, where the actual length is not defined in the type, because we allow arbitrary length.
If the message type is 00, 01, or 10 (Zero byte, 1 byte, 2 byte), then there is no length byte, there is no need.

## Data

The data in Python looks like a string (array of characters), where each character represents a byte. Just do ord(character)
The data on the XMega looks like an array of 8bit values.

Remember, for a zero-byte type, there *is no* data. It just sends a type, a single byte.


## Specification

The messaging works as follows in pseudo-code:

if message_length = 0:
    usart.send(type)
if message_length > 0:
    usart.send(type)
    for character in data:
        usart.send(character)
if message_length is N-Byte Message:
    usart.send(type)
    usart.send(len(data))
    for character in data:
        usart.send(character)

Where [usart](http://en.wikipedia.org/wiki/Universal_asynchronous_receiver/transmitter) is our means of serial communication. 


# TODO
- Add code-generation utility, so that people can define some functions, set some flags, and automatically populate the appropriate files for driver installation
- A Windows test utility (Use the serial_proxy source code)



