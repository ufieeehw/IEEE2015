/*
 * twi.h
 *
 * Created: 12/18/2013 7:04:37 PM
 *  Author: Mason Turner and Khaled Hassan
 *  Code partially based on the Atmel Studio Frameworks implementation of interrupt driven TWI.
 */ 


#ifndef TWI_H_
#define TWI_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "../init.h"


void twi_init_master(void);
void twi_write_byte(uint8_t chip_addr, uint8_t command_addr, uint8_t data);
uint8_t twi_read_byte(uint8_t chip_addr, uint8_t command_addr);
void twi_write_bit(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_num, uint8_t data);
void twi_write_bits(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_start, uint8_t length, uint8_t data);
uint8_t twi_read_bits(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_start, uint8_t length);
int8_t twi_read_bit(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_num);

//Structure for the package that contains data to be transferred
typedef struct
{
	//! TWI chip address to communicate with.
	char chip;
	//! TWI address/commands to issue to the other chip (node).
	uint8_t addr;
	//! Length of the TWI data address segment (1-3 bytes).
	int addr_length;
	//! Where to find the data to be written.
	void *buffer;
	//! How many bytes do we want to write.
	unsigned int length;
	//! Whether to wait if bus is busy (false) or return immediately (true)
	bool no_wait;
} twi_package_t;

//Bus status codes
enum status_code {
	STATUS_OK               =  0, //!< Success
	ERR_IO_ERROR            =  -1, //!< I/O error
	ERR_FLUSHED             =  -2, //!< Request flushed from queue
	ERR_TIMEOUT             =  -3, //!< Operation timed out
	ERR_BAD_DATA            =  -4, //!< Data integrity check failed
	ERR_PROTOCOL            =  -5, //!< Protocol error
	ERR_UNSUPPORTED_DEV     =  -6, //!< Unsupported device
	ERR_NO_MEMORY           =  -7, //!< Insufficient memory
	ERR_INVALID_ARG         =  -8, //!< Invalid argument
	ERR_BAD_ADDRESS         =  -9, //!< Bad address
	ERR_BUSY                =  -10, //!< Resource is busy
	ERR_BAD_FORMAT          =  -11, //!< Data format not recognized

	/**
	 * \brief Operation in progress
	 *
	 * This status code is for driver-internal use when an operation
	 * is currently being performed.
	 *
	 * \note Drivers should never return this status code to any
	 * callers. It is strictly for internal use.
	 */
	OPERATION_IN_PROGRESS	= -128
};
typedef enum status_code status_code_t;

//Structure for a master transfer
static struct transfer
{
	TWI_t *         bus;            // Bus register interface
	twi_package_t * pkg;            // Bus message descriptor
	int             addr_count;     // Bus transfer address data counter
	unsigned int    data_count;     // Bus transfer payload data counter
	bool            read;           // Bus transfer direction
	bool            locked;         // Bus busy or unavailable
	volatile status_code_t status;  // Transfer status

} transfer;

#endif /* TWI_H_ */