#include "init.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "hwlib/uart.h"
#include "hwlib/pid.h"
#include "hwlib/clock.h"
#include "hwlib/twi.h"
#include "hwlib/tcs34725.h"
#include "hwlib/mpu6050.h"
#include "hwlib/hmc5883l.h"

void init() {
	init_clocks();
	init_modules();
	init_interrupts();
	mpu_init();
	imu_init();
	//tcs_init();
	
	uint8_t buffer [12] = {0};
	for(uint8_t i = 0; i < 12; i++)
		buffer[i] = twi_read_byte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_ACCEL_XOUT_H + i);
	int16_t ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	if(ax == 0)
		asm("nop");
	int16_t ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	if(ay == 0)
		asm("nop");
	int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];
	if(az == 0)
		asm("nop");
	int16_t gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	if(gx == 0)
		asm("nop");
	int16_t gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	if(gy == 0)
		asm("nop");
	int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];
	if(gz == 0)
		asm("nop");
	
}

void init_interrupts() {
	PMIC.CTRL = PMIC_RREN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

// All initialization functions for peripherals should be placed here.
void init_modules() {
	uart_init();
	pid_init();
	twi_init_master();
}
