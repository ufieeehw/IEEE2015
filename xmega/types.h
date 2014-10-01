/*Message Types, Mask Definitions, and Macros */
#ifndef _IEEE_TYPES_H_
#define _IEEE_TYPES_H_

//Message Data length is first two bits of type field
#define DATA_MASK             0xC0
#define NO_DATA_TYPE          0x00
#define DATA_1B_TYPE          0x40
#define DATA_2B_TYPE          0x80
#define DATA_NB_TYPE          0xC0


//Error Types must have 11 as bits 6 and 5 (3)
#define ERROR_MASK            0x30
#define ERROR_TYPE            0x30
#define IS_ERROR_TYPE(x)      ((x & ERROR_MASK) == ERROR_TYPE)

//Error Type Definitions (should represent an entire subsystem)
#define OK                    0x00
#define MESSAGE_ERROR_TYPE    0x30
#define BUFFER_ERROR_TYPE     0x31
#define VECTOR_ERROR_TYPE     0xF0

//Example: "JOSH IS COOL" message type, makes host computer print "JOSH IS COOL"
#define JOSHISCOOL_TYPE       0x01



/* TODO: ADD MORE TYPES */

#endif

