#include "init.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "hwlib/uart.h"
#include "hwlib/pid.h"
#include "hwlib/motor.h"
#include "hwlib/clock.h"

void init() {
	init_clocks();
	init_modules();
	init_interrupts();
}

void init_interrupts() {
	PMIC.CTRL = PMIC_RREN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

// All initialization functions for peripherals should be placed here.
void init_modules() {
	uart_init();
	pid_init();
}
