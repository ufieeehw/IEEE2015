//add a mesage to the appropriate section
//register at the start of the range for incoming/mixed message types
//register at the end of the range for outgoing only types
/*Message Types, Mask Definitions, and Macros */
#ifndef _IEEE_TYPES_H_
#define _IEEE_TYPES_H_

/* Masks and Bit Definitons 
The comments afterward are parsed by ROS (ieee2015_xmega_driver/src/xmega_driver/parse_types.py)
White space is not extremely important, but please obey the conventions of "parameter: value"

DO NOT have inline comments on a line with a #define statement if they are not for ROS control.

Byte stride: Number of bytes that contain a single meaningful element. For a 16bit float, byte stride is 2
msg_type: An english word describing the type, should fit into float16, int32 or something like that
expected length: The length of the message that the xmega is expecting for an n-byte message
unit: metric units for the message


*/
//Message Data length is first two bits of type field
//--> Parse types reads this to determine what message length types exist
#define DATA_MASK             0xC0 // mask: data
#define NO_DATA_TYPE          0x00 // msg_length: 0
#define DATA_1B_TYPE          0x40 // msg_length: 1
#define DATA_2B_TYPE          0x80 // msg_length: 2
#define DATA_NB_TYPE          0xC0 // msg_length: None

//Error Types must have 11 as bits 6 and 5
#define ERROR_MASK            0x30 // mask: error
#define ERROR_TYPE            0x30 
#define IS_ERROR_TYPE(x)      ((x & ERROR_MASK) == ERROR_TYPE)


/* Message Type Definitions */
//NO_DATA_TYPE messages [0x01-0x2F]
#define KILL_TYPE             0x01 // out: kill
#define START_TYPE            0x02 // out: start
#define KEEP_ALIVE_TYPE       0x03 // out: keep_alive; in: keep_alive
#define IMU_NOTIFY_TYPE       0x04 // out: poll_imu


//DATA_1B_TYPE messages [0x40-0x6F]
#define DEBUG_TYPE            0x40 // in: xmega_debug; out: ros_debug


//DATA_2B_TYPE messages [0x80-0xAF]
#define STEP_MOTOR_TYPE       0x80 // out: stepper_motor; expected_length: 2; byte_stride: 2; type: integer; unit: steps


//DATA_NB_TYPE messages [0xC0-0xEF]
#define MOTOR_SPEED_TYPE      0xC0 // out: motor_speed; expected_length: 8; byte_stride: 2; type: integer; unit: rad/s
#define IMU_DATA_TYPE         0xEF // in: imu_data; type: non-simple


/*Error Type Definitions (should represent an entire subsystem) */
//NO_DATA_TYPE errors [0x30-0x3F] (and also OK)
#define OK                    0x00 // in: xmega_ok
#define MESSAGE_ERROR_TYPE    0x30 // in: message_error
#define BUFFER_ERROR_TYPE     0x31 // in: buffer_error


//DATA_1B_TYPE errors [0x70-0x7F]


//DATA_2B_TYPE errors [0xB0-0xBF]


//DATA_NB_TYPE errors [0xF0-0xFF]
#define VECTOR_ERROR_TYPE     0xF0 // in: vector_error


/* TODO: FILL IN TYPES */

#endif

