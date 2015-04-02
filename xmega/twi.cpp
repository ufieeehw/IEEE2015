/*
 * twi.cpp
 *
 * Created: 12/18/2013 7:04:25 PM
 *  Author: Mason Turner and Khaled Hassan
 */ 

#include "twi.h"


#define TWI_BAUDRATE(F_TWI) ((F_CPU / (2 * F_TWI)) - 5)

#ifdef __GNUC__
#  define barrier()        asm volatile("" ::: "memory")
#else
#  define barrier()        asm ("")
#endif


// Configuration Values
#define TWI_MASTER_VECT TWID_TWIM_vect
static PORT_t &twi_port = PORTD;
static TWI_t &twi = TWID;
static const uint16_t twi_freq = 50000;



void twi_init_master() {
	// Setup general configuration values.
	twi_port.DIRSET = 0x03;
	twi_port.PIN0CTRL |= PORT_OPC_PULLUP_gc; // enable pullup resistor on SDA
	twi_port.PIN1CTRL |= PORT_OPC_PULLUP_gc; // enable pullup resistor on SCL
	twi.MASTER.BAUD = TWI_BAUDRATE(twi_freq);
	twi.MASTER.CTRLA = TWI_MASTER_INTLVL_LO_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	twi.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
		
	// Set up the transfer struct to have what it needs.
	transfer.locked = false;
	transfer.status = STATUS_OK;
	
}

// Get Exclusive access to global TWI resources
static inline status_code_t twi_master_acquire_bus(bool no_wait) {
	/**
	 * Get exclusive access to global TWI resources.
	 * Wait to acquire bus hardware interface and ISR variables.
	 * param: no_wait,  Set \c true to return instead of doing busy-wait (spin-lock).
	 *
	 * return STATUS_OK if the bus is acquired, else ERR_BUSY.
	 */
		while(transfer.locked) {
			if(no_wait) { return ERR_BUSY; }
		}
		
		transfer.locked = true;
		transfer.status = OPERATION_IN_PROGRESS;
		
		return STATUS_OK;
}

static inline status_code_t twi_master_release_bus(void) {
	/* First wait for the driver event handler to indicate something
	 * other than a transfer in-progress, then test the bus interface
	 * for an Idle bus state.
	 */
	while (OPERATION_IN_PROGRESS == transfer.status);

	while (!((transfer.bus->MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) == TWI_MASTER_BUSSTATE_IDLE_gc))
		barrier(); 

	status_code_t const status = transfer.status;
	transfer.locked = false;

	return status;
}

static status_code_t twi_master_transfer(const twi_package_t *package, bool reading) {
	// Sanity Check.
	if(!package)
		return ERR_INVALID_ARG;
		
	// Initiate transfer when bus is ready.
	status_code_t status = twi_master_acquire_bus(package->no_wait);
	
	if(STATUS_OK == status) {
		transfer.bus		= &twi;
		transfer.pkg		= (twi_package_t *) package;
		transfer.addr_count = 0;
		transfer.data_count = 0;
		transfer.read		= reading;
		
		uint8_t const chip = (package->chip) << 1;
		
		if(package->addr_length || (false == reading)) {
			transfer.bus->MASTER.ADDR = chip;
			} else if(reading) {
			transfer.bus->MASTER.ADDR = chip | 0x01;
		}
		
		status = twi_master_release_bus();
	}
	
	return status;

}

static inline status_code_t twi_master_read(const twi_package_t *package) {
	return twi_master_transfer(package, true);
}

static inline status_code_t twi_master_write(const twi_package_t *package) {
	return twi_master_transfer(package, false);
} 

void twi_write_byte(uint8_t chip_addr, uint8_t command_addr, uint8_t data) {
	uint8_t data_to_send[1] = {data};
	
	twi_package_t packet;
	packet.addr = command_addr;
	packet.addr_length = sizeof(command_addr);
	packet.chip = chip_addr;
	packet.buffer = data_to_send;
	packet.length = sizeof(data_to_send);
	
	while(twi_master_write(&packet) != STATUS_OK);
	
}

uint8_t twi_read_byte(uint8_t chip_addr, uint8_t command_addr) {
	uint8_t data_recieved[1] = {0};
		
	twi_package_t packet_recieved;
	packet_recieved.addr		 = command_addr;
	packet_recieved.addr_length  = 1;
	packet_recieved.chip		 = chip_addr;
	packet_recieved.buffer		 = data_recieved;
	packet_recieved.length		 = sizeof(data_recieved);
	
	while(twi_master_read(&packet_recieved) != STATUS_OK);
	
	return data_recieved[0];
}


/** write a single bit in a 16-bit device register.
 * @param useSPI  true : use SPI 
 * @param devAddr I2C slave device address or Slave Select pin if SPI
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
void twi_write_bit(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_num, uint8_t data){
	uint8_t b = twi_read_byte(chip_addr, command_addr);
	b = (data != 0) ? (b | (1 << bit_num)) : (b & ~(1 << bit_num));
	twi_write_byte(chip_addr, command_addr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param chip_addr I2C slave device address
 * @param command_addr Register regAddr to write to
 * @param bit_start First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
void twi_write_bits(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_start, uint8_t length, uint8_t data){
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 01000000 shift left (8 - length)    ]
    // 00001000 shift right (7 - bitStart) ] --- two shifts ensure all non-important bits are 0
    // 11100011 mask byte
    // 10101111 original value (sample)
    // 10100011 original & mask
    // 10101011 masked | value
	uint8_t b = twi_read_byte(chip_addr, command_addr);
	if (b != 0) {
		//uint8_t mask = (0xFF << (8 - length)) | (0xFF >> (bitStart + length - 1));
		uint8_t mask = (0xFF << (bit_start + 1)) | 0xFF >> ((8 - bit_start) + length - 1);
		data <<= (8 - length);
		data >>= (7 - bit_start);
		b &= mask;
		b |= data;
		twi_write_byte(chip_addr, command_addr, b);	
	}
}

/** Read multiple bits from an 8-bit device register.
 * @param chip_addr I2C slave device address
 * @param command_addr Register regAddr to read from
 * @param bit_start First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @return bits read
 */
uint8_t twi_read_bits(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_start, uint8_t length) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
	
	uint8_t b = twi_read_byte(chip_addr, command_addr);
	uint8_t r = 0;
	if (b != 0) {
		for (uint8_t i = bit_start; i > bit_start - length; i--) {
			r |= (b & (1 << i));
		}
		r >>= (bit_start - length + 1);
		return r;
	}
	return -1;	
}

/** Read a single bit from an 8-bit device register.
 * @param chip_addr I2C slave device address
 * @param command_addr Register regAddr to read from
 * @param bit_num Bit position to read (0-7)
 * @return bit
 */
int8_t twi_read_bit(uint8_t chip_addr, uint8_t command_addr, uint8_t bit_num) {
	uint8_t b = twi_read_byte(chip_addr, command_addr);
	return b & (1 << bit_num);
}

//! TWI master write interrupt handler.
static inline void twim_write_handler(void) {
	TWI_t *const			bus = transfer.bus;
	twi_package_t *const	pkg = transfer.pkg;
	
	if(transfer.addr_count < pkg->addr_length) {
		//If the number of addresses dealt with so far
		//is less than the total number, then write the
		const uint8_t * const data = &((*pkg).addr); //pkg->addr;
		bus->MASTER.DATA = data[transfer.addr_count++];
		
		} else if(transfer.data_count < pkg->length) {
		if(transfer.read) {
			//Send repeated START condition (Address|R/W=1)
			bus->MASTER.ADDR |= 0x01;
			} else {
			uint8_t * const data = (uint8_t *)((*pkg).buffer); //pkg->buffer;
			bus->MASTER.DATA = data[transfer.data_count++];
		}
		} else {
		// Send STOP condition to complete the transaction
		bus->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		transfer.status = STATUS_OK;
	}
}

//! TWI master read interrupt handler.
static inline void twim_read_handler(void) {
	TWI_t * const			bus = transfer.bus;
	twi_package_t * const	pkg = transfer.pkg;
	
	if(transfer.data_count < pkg->length) {
		uint8_t * const data = (uint8_t *)((*pkg).buffer); //pkg->buffer;
		data[transfer.data_count++] = bus->MASTER.DATA;
		
		/* If there is more to read, issue ACK and start a byte read.
		 * Otherwise, issue NACK and STOP to complete the transaction.
		 */
		if(transfer.data_count < pkg->length) {
			bus->MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
		} else {
			bus->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
			transfer.status = STATUS_OK;
		}
	} else {
		//Issue STOP and buffer overflow condition
		bus->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		transfer.status = ERR_NO_MEMORY;
	}
}

//! Common TWI master interrupt service routine.
static void twim_interrupt_handler(void) {
	uint8_t const master_status = transfer.bus->MASTER.STATUS;
	
	if(master_status & TWI_MASTER_ARBLOST_bm) {
		transfer.bus->MASTER.STATUS = master_status | TWI_MASTER_ARBLOST_bm;
		transfer.bus->MASTER.CTRLC  = TWI_MASTER_CMD_STOP_gc;
		transfer.status = ERR_BUSY;
	} else if((master_status & TWI_MASTER_BUSERR_bm) || (master_status & TWI_MASTER_RXACK_bm)) {
		transfer.bus->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		transfer.status = ERR_IO_ERROR;
	} else if(master_status & TWI_MASTER_WIF_bm) {
		twim_write_handler();
	} else if(master_status & TWI_MASTER_RIF_bm) {
		twim_read_handler();
	} else {
		transfer.status = ERR_PROTOCOL;
	}
}

ISR(TWI_MASTER_VECT) {
	 twim_interrupt_handler(); 
}