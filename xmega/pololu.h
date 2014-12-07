
#ifndef POLOLU_DRIVER_H_INCLUDED
#define POLOLU_DRIVER_H_INCLUDED

#include <asf.h>

typedef struct {
	PORT_t *PORT;
	TC2_t * TC2;
	Bool motor2;
	} pololu_t;

void pololuInit(pololu_t *pololu);
void pololu_set_effort(pololu_t *pololu, float new_effort);
void pololu_set_velocity(pololu_t *pololu, int16_t velocity);

#endif // POLOLU_DRIVER_H