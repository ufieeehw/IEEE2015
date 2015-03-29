
#ifndef POLOLU_DRIVER_H_INCLUDED
#define POLOLU_DRIVER_H_INCLUDED

typedef struct {
	PORT_t *PORT;
	TC1_t * TC;
} pololu_high_t;

typedef struct {
	PORT_t *PORT;
	TC0_t * TC;
} pololu_low_t;

void pololuInit(pololu_high_t *high);
void pololuInit(pololu_low_t *low);
void pololu_set_effort(pololu_high_t *high, float new_effort);
void pololu_set_effort(pololu_low_t *low, float new_effort);
void pololu_set_velocity(pololu_high_t *high, float velocity);
void pololu_set_velocity(pololu_low_t *low, float velocity);

#endif // POLOLU_DRIVER_H