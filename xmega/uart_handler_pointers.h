/*
 * uart_handler_pointers.h
 *
 * Created: 1/11/2014 9:49:32 PM
 *  Author: Mason
 */ 


#ifndef UART_HANDLER_POINTERS_H_
#define UART_HANDLER_POINTERS_H_

#include "uart.h"
#include "pid.h"
#include "clock.h"


typedef void (*HandlerPointer)(char*, uint8_t);
HandlerPointer HandlerPointers[10] = {
	NULL,						// 0x00 AckValid
	NULL,						// 0x01 AckInvalid
	uart_echo_request,			// 0x02
	uart_echo_reply,			// 0x03
	pid_set_speed_handler,		// 0x04
	pid_get_odometry_handler,	// 0x05
	pid_set_speed_multiplier_handler, // 0x06
	pid_get_speed_multiplier_handler, // 0x07
	pid_get_speed_handler,		// 0x08
	pid_set_tuning_handler,		//0x09
};



#endif /* UART_HANDLER_POINTERS_H_ */