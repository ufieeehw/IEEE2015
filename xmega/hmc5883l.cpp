/*
 * hmc5883l.cpp
 *
 * Created: 4/4/2015 2:18:31 AM
 *  Author: Ian
 */ 
#include <avr/io.h>
#include "hmc5883l.h"
#include "twi.h"

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

void imu_init(){
	// Let Jesus take the keyboard, because only God knows what these registers do.
	// write CONFIG_A register
	twi_write_byte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
	(HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
	(HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
	(HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));
	// write CONFIG_B register
	imu_setGain(HMC5883L_GAIN_1090);
	// write MODE register
	imu_setMode(HMC5883L_MODE_SINGLE);
}

//Each value ranges fom -2048 to 2047
/*
void imu_getHeading(int16_t *x, int16_t *y, int16_t *z) {
	//I2Cdev::readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) 
		twi_write_byte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[4]) << 8) | buffer[5];
	*z = (((int16_t)buffer[2]) << 8) | buffer[3];
}
*/

