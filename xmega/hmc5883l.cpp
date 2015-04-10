/*
 * hmc5883l.cpp
 *
 * Created: 4/4/2015 2:18:31 AM
 *  Author: Ian
 */ 
#include <avr/io.h>
#include "hmc5883l.h"
#include "twi.h"
#include "../init.h"
#include "uart.h"

static uint8_t mode = HMC5883L_MODE_SINGLE;

static inline void imu_setGain(uint8_t gain) {
	// use this method to guarantee that bits 4-0 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	twi_write_byte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
}

static inline void imu_setMode(uint8_t newMode) {
	// use this method to guarantee that bits 7-2 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	twi_write_byte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, newMode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mode = newMode; // track to tell if we have to clear bit 7 after a read
}

static inline void imu_setSampleAveraging(uint8_t averaging) {
	twi_write_bits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}

static inline void imu_setDataRate(uint8_t rate) {
	twi_write_bits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}

static inline void imu_setMeasurementBias(uint8_t bias) {
	twi_write_bits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

void imu_init(){
	// Let Jesus take the keyboard, because only God knows what these registers do.
	// write CONFIG_A register
	twi_write_byte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
	(HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
	(HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
	(HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));
	// write CONFIG_B register
	imu_setGain(HMC5883L_GAIN_440);
	// write MODE register
	imu_setMode(HMC5883L_MODE_SINGLE);
	
	imu_setSampleAveraging(HMC5883L_AVERAGING_8);
	
	imu_setDataRate(HMC5883L_RATE_75);
	
	imu_setMeasurementBias(HMC5883L_BIAS_NORMAL);
}

//Each value ranges from -2048 to 2047
void imu_getHeading(char (&buffer)[6]) {
	for (uint8_t i = 0; i < 7; i++)
		buffer[i] = twi_read_byte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H + i);
	if (mode == HMC5883L_MODE_SINGLE) 
		twi_write_byte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
}

void imu_get_heading_handler(char* message, uint8_t len){
	char buffer[6] = {0};
	imu_getHeading(buffer);
	uart_send_msg_block(IMUgetHeading, buffer, 7);
}