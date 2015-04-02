/*
 * tcs34725.cpp
 *
 * Created: 12/19/2013 12:02:49 AM
 *  Author: Khaled
 */ 

#include <avr/io.h>
#include "tcs34725.h"
#include "uart.h"

static bool _tcs34725Initialised;
static tcs34725Gain_t _tcs34725Gain;
static tcs34725IntegrationTime_t _tcs34725IntegrationTime;
static PORT_t &twi_port = PORTD;

static inline void write8(uint8_t reg, uint8_t value){
	twi_write_byte(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT | reg), value);
}

static inline uint8_t read8(uint8_t reg){
	return twi_read_byte(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT | reg));
}


static uint16_t read16(uint8_t reg){
	uint8_t low = read8(reg);
	uint8_t high = read8(reg+1);
	
	return ( ((uint16_t)high << 8) | (uint16_t) low );
}


bool tcs_init(void){
	twi_port.DIRSET = _BV(2); // set LED control pin to output
	twi_port.OUTCLR = _BV(2); // set LED control pin to default low
	uint8_t id = read8(TCS34725_ID);
	
	if(id != TCS34725_EXPECTED_ID){
		return false;
	}
	
	_tcs34725Initialised = true;
	
	/* Set default integration time and gain, as per Adafruit_TCS34725.h */
	/* Are these really the defaults? */ 
	tcs_setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
	tcs_setGain(TCS34725_GAIN_16X);
	/*
	tcs_setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
	tcs_setGain(TCS34725_GAIN_4X);
	*/
	
	/* Note: by default, the device is in power down mode on bootup */
	write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
	_delay_ms(3);
	write8(TCS34725_ENABLE, (TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
	
	return true;
}

void tcs_setIntegrationTime(tcs34725IntegrationTime_t it){
	if (!_tcs34725Initialised) tcs_init();

	/* Update the timing register */
	write8(TCS34725_ATIME, it);

	/* Update value placeholders */
	_tcs34725IntegrationTime = it;
}

void tcs_setGain(tcs34725Gain_t gain){
	if (!_tcs34725Initialised) tcs_init();

	/* Update the timing register */
	write8(TCS34725_CONTROL, gain);

	/* Update value placeholders */
	_tcs34725Gain = gain;
}


void tcs_getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c){
	if (!_tcs34725Initialised) tcs_init();

	*c = read16(TCS34725_CDATAL);
	*r = read16(TCS34725_RDATAL);
	*g = read16(TCS34725_GDATAL);
	*b = read16(TCS34725_BDATAL);
	
	/* Set a delay for the integration time */
	switch (_tcs34725IntegrationTime)
	{
		case TCS34725_INTEGRATIONTIME_2_4MS:
		_delay_ms(3);
		break;
		case TCS34725_INTEGRATIONTIME_24MS:
		_delay_ms(24);
		break;
		case TCS34725_INTEGRATIONTIME_50MS:
		_delay_ms(50);
		break;
		case TCS34725_INTEGRATIONTIME_101MS:
		_delay_ms(101);
		break;
		case TCS34725_INTEGRATIONTIME_154MS:
		_delay_ms(154);
		break;
		case TCS34725_INTEGRATIONTIME_700MS:
		_delay_ms(700);
		break;
	}
}

uint16_t tcs_calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b){
	float X, Y, Z;      /* RGB to XYZ correlation      */
	float xc, yc;       /* Chromaticity co-ordinates   */
	float n;            /* McCamy's formula            */
	float cct;

	/* 1. Map RGB values to their XYZ counterparts.    */
	/* Based on 6500K fluorescent, 3000K fluorescent   */
	/* and 60W incandescent values for a wide range.   */
	/* Note: Y = Illuminance or lux                    */
	X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
	Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
	Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

	/* 2. Calculate the chromaticity co-ordinates      */
	xc = (X) / (X + Y + Z);
	yc = (Y) / (X + Y + Z);

	/* 3. Use McCamy's formula to determine the CCT    */
	n = (xc - 0.3320F) / (0.1858F - yc);

	/* Calculate the final CCT */
	cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

	/* Return the results in degrees Kelvin */
	return (uint16_t)cct;
}

uint16_t tcs_calculateLux(uint16_t r, uint16_t g, uint16_t b){
	float illuminance;

	/* This only uses RGB ... how can we integrate clear or calculate lux */
	/* based exclusively on clear since this might be more reliable?      */
	illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

	return (uint16_t)illuminance;
}

void tcs_setInterrupt(bool i) {
	uint8_t r = read8(TCS34725_ENABLE);
	if (i) {
		r |= TCS34725_ENABLE_AIEN;
		} else {
		r &= ~TCS34725_ENABLE_AIEN;
	}
	write8(TCS34725_ENABLE, r);
}

void tcs_get_raw_data_handler(char* messsage, uint8_t len) {
	char buffer[8];
	uint16_t colorVals[4];
	tcs_getRawData(&colorVals[0], &colorVals[1], &colorVals[2], &colorVals[3]); // r, g, b, c

	for(int i = 0; i < 8; i+=2){
		buffer[i] = (colorVals[i>>1] & 0xFF);
		buffer[i+1] = ((colorVals[i>>1] >> 8) & 0xFF);
	}
	uart_send_msg_block(TCSGetRawData, buffer, 9);
}

/* TODO: implement once we can write a raw byte using TWI

void tcs_clearInterrupt(void) {
	Wire.beginTransmission(TCS34725_ADDRESS);
	#if ARDUINO >= 100
	Wire.write(0x66);
	#else
	Wire.send(0x66);
	#endif
	Wire.endTransmission();
}
*/