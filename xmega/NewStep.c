/*
	Message expects 2 byte int value between -1800 to 1800 
	This value corresponds to the target angle * 10
		I am type 0x80
*/

#include "types.h"
#include "message.h"
#include <avr/io.h>

//F_CPU needs to be decalred to use delay.h
#define F_CPU 32000000UL 
#include <util/delay.h>


static float currentAngle;

#define STEPSINFULLROUND 3200
#define TOTALSTEPS 200
#define microSteps 1	//1/32 microsteps
#define BIT(x) 			(1 << (x)) 
#define SETBITS(x,y) 	((x) |= (y)) 
#define CLEARBITS(x,y) 	((x) &= (~(y))) 
#define SETBIT(x,y) 	SETBITS((x), (BIT((y))))			/* EXAMPLE SETBIT(PORTB,2) sets the 2 bit of PORTB */
#define CLEARBIT(x,y) 	CLEARBITS((x), (BIT((y)))) 
#define BITSET(x,y) 	((x) & (BIT(y))) 
#define BITCLEAR(x,y) 	!BITSET((x), (y)) 
#define BITSSET(x,y) 	(((x) & (y)) == (y)) 
#define BITSCLEAR(x,y) 	(((x) & (y)) == 0) 
#define BITVAL(x,y) 	(((x)>>(y)) & 1) 
#define ENABLEPIN 2
#define DIRPIN 3
#define STEPPIN 4
#define RIGHT 1
#define PORT PORTC_OUT
#define LEFT 0
void step(float targetAngle, int direction){
	
	if(targetAngle==currentAngle){
		//skips 
	}
	else{
		if(direction==RIGHT) SETBIT(PORT, DIRPIN);
		else CLEARBIT(PORT, DIRPIN);
		/*
		if(getDirStep(targetAngle)){ SETBIT(PORTC_OUT, DIRPIN);}
		else {CLEARBIT(PORTC_OUT, DIRPIN);}
			*/
		//find how many steps to take to target degree
		float driveStep = targetAngle-currentAngle;
		if(driveStep<0) driveStep = driveStep*-1;

		//update currentAngle
		if(direction == RIGHT){ currentAngle = currentAngle+driveStep;}
		else{currentAngle = currentAngle-driveStep;}

		//convert angle to steps
		driveStep = (driveStep/360)*STEPSINFULLROUND;

		//drive step
		for(int i =0; i<(int)driveStep; i++){
			SETBIT(PORT, STEPPIN);
			//holdHalfCycle(speedRPS);
			_delay_us(500);
			CLEARBIT(PORT, STEPPIN);
			//holdHalfCycle(speedRPS);
			_delay_us(500);
		}
	}
	
}
void resetAngle(){
	currentAngle = 0;
}
void initStep(){
	//initialize I/O port here
	// PORTC_DIR = 0xff;
	// CLEARBIT(PORT, ENABLEPIN);	//turn on motor
	// SETBIT(PORT, DIRPIN);
	// currentAngle = 0;
}

int stepMotorMessage(Message m){
	int16_t x = *m.data;
	float targetAngle = x/10;
	step(targetAngle, RIGHT);
	return OK;
}
