/*
 * CFile1.c
 *
 * Created: 10/24/2014 6:19:38 PM
 *  Author: Ian
 */ 
/**
 * Each motor uses 4 pins, which can be on any port with a timer counter.
 * Pint out:
 * Low:
 * Pin0 = M1EN
 * Pin1 = M1INA
 * Pin2 = M1INB
 * Pin3 = M1PWM
 *************
 * There is no PWM output on pin 7, so the layout is mirrored on the high pins
 * High:
 * Pin4 = M2PWM 
 * Pin5 = M2INB
 * Pin6 = M2INA
 * Pin7 = M2EN
 * 
 */

#include <avr/io.h>
#include <math.h>
#include "pololu.h"

#define PERIOD  1664

void pololuInit(pololu_high_t *high){
	high->PORT->DIRSET = 0xF0; //Repeat for motor on high pins of port
	high->PORT->OUTCLR = 0xF0; //Inputs are mirrored
	high->PORT->PIN7CTRL = 0x18; //Sets pin to have a pull up
	high->PORT->OUTSET = 0x80;
	
	high->TC->CTRLA = TC_CLKSEL_DIV1_gc;
	high->TC->CTRLB = TC1_CCAEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	
	high->TC->PER = PERIOD;
	high->TC->CCA = 0;
	pololu_set_velocity(high,0);
	
}

void pololuInit(pololu_low_t *low){
	low->PORT->DIRSET = 0x0F; //Set MxINA, MxINB, MxPWM, and MxEN as outputs
	low->PORT->OUTCLR = 0x0F; //Set MxINA, MxINB, and MxPWM low
	low->PORT->PIN0CTRL = 0x18; //Sets pin to have a pull up
	low->PORT->OUTSET = 0x01; //Set MxEN high.
	
	low->TC->CTRLA = TC_CLKSEL_DIV8_gc;
	low->TC->CTRLB = TC0_CCDEN_bm | TC_WGMODE_SINGLESLOPE_gc;

	low->TC->PER = PERIOD;
	low->TC->CCD = 0;
	pololu_set_velocity(low,0);
}

//1024 max value preserved for compatibility.
void pololu_set_effort(pololu_high_t *high, float new_effort){
	high->TC->CCA = (PERIOD * (new_effort/1024.0));
}

void pololu_set_effort(pololu_low_t *low, float new_effort){
	low->TC->CCD = (PERIOD * (new_effort/1024.0));
}

void pololu_set_velocity(pololu_high_t *high, float velocity){
	//Counter clockwise
	if(velocity < 0){
		high->PORT->OUTCLR = 0x40; //Set INA to 0
		high->PORT->OUTSET = 0x20; //Set INB to 1
	}
	//Clockwise
	else{
		high->PORT->OUTSET = 0x40; //Set INA to 1
		high->PORT->OUTCLR = 0x20; //Set INB to 0
	}
	volatile float speed = fabs(velocity);
	if(speed >= 1024) speed = 1024;
	pololu_set_effort(high, speed);
}

void pololu_set_velocity(pololu_low_t *low, float velocity){
	//Counter clockwise
	if(velocity < 0){
		low->PORT->OUTCLR = 0x04; //Set INA to 0
		low->PORT->OUTSET = 0x02; //Set INB to 1
	}
	//Clockwise
	else{
		low->PORT->OUTSET = 0x04; //Set INA to 1
		low->PORT->OUTCLR = 0x02; //Set INB to 0
	}
	volatile float speed = fabs(velocity);
	if(speed >= 1023) speed = 1023;
	pololu_set_effort(low, speed);
}