/*
 * CFile1.c
 *
 * Created: 10/24/2014 6:19:38 PM
 *  Author: Ian
 */ 
/**
 * Each motor uses 4 pins, which can be on any port with a timer counter.
 * Pint out:
 * !motor2:
 * Pin0 = M1EN
 * Pin1 = M1INA
 * Pin2 = M1INB
 * Pin3 = M1PWM
 *************
 * motor2:
 * Pin4 = M2EN
 * Pin5 = M2INA
 * Pin6 = M2INB
 * Pin7 = M2PWM
 * 
 */

#include <avr/io.h>
#include <math.h>
#include "pid.h"
#include "pololu.h"

#define PERIOD  208

void pololuInit(pololu_t *pololu){
	int8_t shift = 0;
	if (pololu->motor2) //Shift right 4 if setting up motor2
		shift = 4;
	pololu->PORT->DIRSET = 0x0F << shift; //Set MxINA, MxINB, MxPWM, and MxEN as outputs
	pololu->PORT->OUTCLR = 0x0E << shift; //Set MxINA, MxINB, and MxPWM low
	pololu->PORT->OUTSET = 0x01; //Set MxEN high.
	pololu->TC2->CTRLA |= TC_CLKSEL_DIV8_gc;
	pololu->TC2->CTRLE |= TC_BYTEM_SPLITMODE_gc;
	
	if (pololu->motor2){
		pololu->TC2->CTRLB |= TC2_HCMPDEN_bm;	//Enable compare on upper or lower bits
		pololu->TC2->HPER = PERIOD;
	}
	else{
		pololu->TC2->CTRLB |= TC2_LCMPDEN_bm;	 
		pololu->TC2->LPER = PERIOD;
	}
		
}


//1024 max value preserved for compatibility.
void pololu_set_effort(pololu_t *pololu, float new_effort){
	
	volatile int16_t checkThis = (int16_t)new_effort;
	
	if(pololu->motor2)
		pololu->TC2->HCMPD = (PERIOD * (new_effort/1024.0)); //Same as setting CCx when using TC0/1
	else
		pololu->TC2->LCMPD = (PERIOD * (new_effort/1024.0));
}

void pololu_set_velocity(pololu_t *pololu, int16_t velocity){
	uint8_t shift = 0;
	if(pololu->motor2)
		shift = 4;
	//Counter clockwise
	if(velocity < 0){
		pololu->PORT->OUTCLR = 0x02 << shift; //Set INA to 0
		pololu->PORT->OUTSET = 0x04 << shift; //Set INB to 1
	}
	//Clockwise
	else{
		pololu->PORT->OUTSET = 0x02 << shift; //Set INA to 1
		pololu->PORT->OUTCLR = 0x04 << shift; //Set INB to 0
	}
	
	volatile float speed = fabs(velocity);
	if(speed >= 1024) speed = 1024;
	pololu_set_effort(pololu, speed);
	}