/*Message Types, Mask Definitions, and Macros */
#ifndef _IEEE_TYPES_H_
#define _IEEE_TYPES_H_

//Message Data length is first two bits of type field
#define DATA_MASK             0xC0
#define NO_DATA_TYPE          0x00
#define DATA_1B_TYPE          0x40
#define DATA_2B_TYPE          0x80
#define DATA_NB_TYPE          0xC0

//Error Handling Types
#define OK                    0x00
#define QUEUE_FULL_TYPE       0x30
#define QUEUE_EMPTY_TYPE      0x31

//"JOSH IS COOL" message type, makes host computer print "JOSH IS COOL"
#define JOSHISCOOL_TYPE       0xFE



/* TODO: ADD MORE TYPES */

#endif

