#ifndef STEPPER_H
#define STEPPER_H

#include "stdint.h"

typedef struct
{
	uint8_t direction;
	uint32_t speed;
	uint64_t step_delay,last_step_time;
	uint8_t pin_count;
	uint8_t step_number;
	uint8_t p1,p2,p3,p4;
} stepper_t, *stepper;

stepper_t stepper2(uint8_t,uint8_t);
stepper_t stepper4(uint8_t,uint8_t,uint8_t,uint8_t);
void setSpeed(stepper_t *,uint32_t);
void step(stepper_t *,uint8_t);
void stepMotor(stepper_t *,uint8_t);

#endif
