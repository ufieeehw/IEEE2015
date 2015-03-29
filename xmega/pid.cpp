#include "pid.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "uart.h"
#include "../init.h"
#include "pololu.h"

// Configuration Values
static TC0_t &pid_tick_timer = TCC0;
#define PID_TICK_OVF TCC0_OVF_vect

static int constP = 120;
static int constI = 2950;
static float constD = 0.0004;


static PORT_t &wheelPort1 = PORTA;
static PORT_t &wheelPort2 = PORTB;


static float radPerTick = (2*M_PI)/1856.0;
static const float sampleTime = 10e-3;

static pololu_low_t pololu_LF = {
	.PORT = &PORTF,
	.TC = &TCF0,
};
static pololu_high_t pololu_LR = {
	.PORT = &PORTF,
	.TC = &TCF1,
};
static pololu_low_t pololu_RF = {
	.PORT = &PORTE,
	.TC = &TCE0,
};
static pololu_high_t pololu_RR = {
	.PORT = &PORTE,
	.TC = &TCE1,
};


// File-Scope Variables and Structures
struct pid_wheel_data {
	
	// Values for measuring the speed of the wheel
	volatile float AVG_speed = 0;
	volatile int32_t ticks = 0;
	
	// Values for the actual pid controller
	float input = 0, output = 0, setpoint = 0, errSum = 0, lastErr = 0, kp = 0, ki = 0, kd = 0;

	int32_t pid_last_ticks = 0;
	int32_t odometry_last_ticks = 0;
} ;

static pid_wheel_data wheelData[4];

void pid_init() {
	
	pololuInit(&pololu_LF);
	pololuInit(&pololu_LR);
	pololuInit(&pololu_RF);
	pololuInit(&pololu_RR);
	
	// Initialize motor frequency measurement timers.
	wheelPort1.DIRCLR = 0x06;
	wheelPort1.PIN2CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort1.INT0MASK = 0x14;
	wheelPort1.INTCTRL = 0x05;

	wheelPort1.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort1.INT1MASK = 0x0A;
	
	wheelPort2.DIRCLR = 0x03;
	wheelPort2.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort2.INT0MASK = 0x05;
	wheelPort2.INTCTRL = 0x05;

	wheelPort2.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort2.INT1MASK = 0x0A;

	
	// Initialize the PID tick timer
	pid_tick_timer.CTRLA = TC_CLKSEL_DIV64_gc;	//Uses timer on portC since all timers on PORT E and D will be used for wheels.
	pid_tick_timer.CTRLB = 0x00;
	pid_tick_timer.CTRLC = 0x00;
	pid_tick_timer.CTRLD = 0x00;
	pid_tick_timer.CTRLE = 0x00;
	pid_tick_timer.PER = round(F_CPU * sampleTime / 64.);
	pid_tick_timer.INTCTRLA = TC_OVFINTLVL_LO_gc;

	pid_setTunings(constP, constI, constD, WHEEL_RF);
	pid_setTunings(constP, constI, constD, WHEEL_RR);	
	pid_setTunings(constP, constI, constD, WHEEL_LF);
	pid_setTunings(constP, constI, constD, WHEEL_LR);

	
	////TEST code for setting a default speed, remove when done
	for(int i = 0; i < 4; i++) pid_setSpeed(500, (wheelNum)i);
	
}

static void pid_compute(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	
	// Compute all working error variables
	float error = data.setpoint - data.AVG_speed;
	data.errSum += error;
	float dErr = (error - data.lastErr);
	
	if(data.ki * data.errSum >= 1024) {
		data.errSum = 1024/data.ki;
		} else if(data.ki * data.errSum <= -1024) {
		data.errSum = -1024/data.ki;
	}
	
	//Compute the output
	data.output = (data.kp * error) + (data.ki * data.errSum) + (data.kd * dErr);
	
	//Remember some things for later
	data.lastErr = error;
}

void pid_setTunings(float Kp, float Ki, float Kd, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	data.kp = Kp;
	data.ki = Ki*sampleTime;
	data.kd = Kd/sampleTime;
}

static void pid_measureSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	
	cli();
	int32_t ticks = data.ticks;
	sei();
	
	data.AVG_speed = (ticks - data.pid_last_ticks)*radPerTick/sampleTime;
	
	data.pid_last_ticks = ticks;
}

float pid_getSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	return data.AVG_speed;
}

void pid_setSpeed(float speed, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	data.setpoint = speed;
}

/*******************************************************************
-----> Handler Functions <-----
*******************************************************************/

// Handler that sets wheel speeds based on message.
// Arguments:
//	message := [Wheel1B0, Wheel1B1, Wheel1B2, Wheel1B3, ... ,  Wheel4B0, Wheel4B1, Wheel4B2, Wheel4B3]
//		16 bytes, little endian 32bit numbers that represent the desired wheel speed
//		multiplied by 1000.  Pass a pointer to the low byte of
//	len := the length of the message.  E.g. the number of bytes in the array
void pid_set_speed_handler(char* message, uint8_t len) {
	cli();

	for(int i = 0; i < 4; i++) pid_setSpeed(uart_int32_to_float(&message[4*i]), (wheelNum)i);
	
	sei();
}

void pid_get_speed_handler(char* message, uint8_t len) {
	char wheelSpeeds[16];
	uart_float_to_char32((char*)&wheelSpeeds[8], pid_getSpeed(WHEEL_RF));
	uart_float_to_char32((char*)&wheelSpeeds[12], pid_getSpeed(WHEEL_RR));	
	uart_float_to_char32((char*)&wheelSpeeds[0], pid_getSpeed(WHEEL_LF));
	uart_float_to_char32((char*)&wheelSpeeds[4], pid_getSpeed(WHEEL_LR));

	
	uart_send_msg_block(PIDgetSpeed, wheelSpeeds, 17);
}

void pid_set_tuning_handler(char* message, uint8_t len){
	constP = (int)(message[0]);
	constI = uart_int32_to_float(&message[8])/10000;
	constD = (int)(message[4]);
	pid_setTunings(constP, constI, constD, WHEEL_RF);
	pid_setTunings(constP, constI, constD, WHEEL_RR);	
	pid_setTunings(constP, constI, constD, WHEEL_LF);
	pid_setTunings(constP, constI, constD, WHEEL_LR);

}


static void pid_get_odometry(float* returnData) {
	for(int i = 0; i < 4; i++) {
		pid_wheel_data &data = wheelData[i];
		
		cli();
		int32_t ticks = data.ticks;
		sei();
		
		volatile int32_t tmp = ticks - data.odometry_last_ticks;
		volatile float tmp2 = tmp * radPerTick;
		
		returnData[i] = tmp2;
		
		data.odometry_last_ticks = ticks;
	}
}

void pid_get_odometry_handler(char* message, uint8_t len) {

	char odometry[16];
	float wheelOdometry[4];
	pid_get_odometry(wheelOdometry);
	
	for(int i = 0; i < 4; i++) {
		uart_float_to_char32(odometry+i*4, wheelOdometry[i]);
	}

	uart_send_msg_block(PIDgetOdometry, odometry, 17);

}

static inline void pid_set_speed_multiplier(float val) {
	radPerTick = val;
}

static inline float pid_get_speed_multiplier() {
	return radPerTick;
}

// Because the multiplier is a floating point value, we'll multiply it by 1000 first, and then send it.
// Or, if we're receiving it, we'll divide it by 1000.
void pid_get_speed_multiplier_handler(char* message, uint8_t len) {
	char multiplier[2];
	uart_float_to_char16(multiplier, pid_get_speed_multiplier());
	uart_send_msg_block(PIDgetMultiplier, multiplier, 3);
}

void pid_set_speed_multiplier_handler(char* message, uint8_t len) {
	pid_set_speed_multiplier(uart_int16_to_float(message));
}

int count = 0;
ISR(PID_TICK_OVF) {
	pid_measureSpeed(WHEEL_RF);
	pid_measureSpeed(WHEEL_RR);
	pid_measureSpeed(WHEEL_LF);
	pid_measureSpeed(WHEEL_LR);

	pid_compute(WHEEL_RF);
	pid_compute(WHEEL_RR);
	pid_compute(WHEEL_LF);
	pid_compute(WHEEL_LR);
	pololu_set_velocity(&pololu_RF, wheelData[WHEEL_RF].output);
	pololu_set_velocity(&pololu_RR, wheelData[WHEEL_RR].output);
	pololu_set_velocity(&pololu_LF, wheelData[WHEEL_LF].output);
	pololu_set_velocity(&pololu_LR, wheelData[WHEEL_LR].output);

}

unsigned int grayToBinary(unsigned int num)
{
	unsigned int mask;
	for (mask = num >> 1; mask != 0; mask = mask >> 1)
	{
		num = num ^ mask;
	}
	return num;
}


//Left Front
ISR(PORTA_INT0_vect){
	// pins 2 and 4
	static int8_t old_value = 0;
	//int8_t new_value = grayToBinary(((wheelPort2.IN & 4) >> 1) | (wheelPort2.IN & 1));
	int8_t new_value = grayToBinary(((wheelPort1.IN & 0x10) >> 3) | (wheelPort1.IN & 0x04) >> 2);
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[WHEEL_LF].ticks += difference;
	
	old_value = new_value;
	
}

//Left Rear
ISR(PORTA_INT1_vect){
	
	// pins 1 and 3
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort1.IN & 8) >> 2) | (wheelPort1.IN & 2) >> 1);
	//int8_t new_value = grayToBinary(((wheelPort1.IN & 8) >> 1) | (wheelPort1.IN & 2));
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[WHEEL_LR].ticks += difference;
	
	old_value = new_value;
}

//Right Front
ISR(PORTB_INT0_vect){
	// pins 0 and 2
	static int8_t old_value = 0;
	//int8_t new_value = grayToBinary(((wheelPort2.IN & 4) >> 1) | (wheelPort2.IN & 1));
	int8_t new_value = grayToBinary(((wheelPort2.IN & 4) >> 1) | (wheelPort2.IN & 1));
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[WHEEL_RF].ticks += difference;
	
	old_value = new_value;
}

//Right Rear
ISR(PORTB_INT1_vect){
	// pins 1 and 3
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort2.IN & 8) >> 2) | (wheelPort2.IN & 2) >> 1);
	//int8_t new_value = grayToBinary(((wheelPort2.IN & 8) >> 1) | (wheelPort2.IN & 2));
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[WHEEL_RR].ticks += difference;
	
	old_value = new_value;
}


