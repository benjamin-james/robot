#include "avr_util.h"
#include "stepper.h"
#include "stdint.h"

stepper_t stepper2(uint8_t pin1, uint8_t pin2)
{
	stepper_t s = {0,0,0,0,2,0,pin1,pin2,0,0};
	pinMode(pin1,OUTPUT);
	pinMode(pin2,OUTPUT);
	return s;
}
stepper_t stepper4(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
	stepper_t s = {0,0,0,0,4,0,pin1,pin2,pin3,pin4,pin5};
	pinMode(pin1,OUTPUT);
	pinMode(pin2,OUTPUT);
	pinMode(pin3,OUTPUT);
	pinMode(pin4,OUTPUT);
	return s;
}
void stepper_setSpeed(stepper_t *s, uint32_t speed)
{
	if(s) s->step_delay = 15000 / speed;//60*1000/4 -> seconds->milliseconds->per step
}
void stepper_stepMotor(stepper_t *s, uint8_t stepnum)
{
	if(!s) return;
	if(s->pin_count == 2)
	{
		switch(stepnum)
		{
			case 0:
			digitalWrite(s->p1,LOW);
			digitalWrite(s->p2,HIGH);
			break;
			case 1:
			digitalWrite(s->p1,HIGH);
			digitalWrite(s->p2,HIGH);
			break;
			case 2:
			digitalWrite(s->p1,HIGH);
			digitalWrite(s->p2,LOW);
			break;
			case 3:
			digitalWrite(s->p1,LOW);
			digitalWrite(s->p2,LOW);
			break;
		}
	}
	else if(s->pin_count == 4)
	{
		switch(stepnum)
		{
			case 0:
			digitalWrite(s->p1,HIGH);
			digitalWrite(s->p2,LOW);
			digitalWrite(s->p3,HIGH);
			digitalWrite(s->p4,LOW);
			break;
			case 1:
			digitalWrite(s->p1,LOW);
			digitalWrite(s->p2,HIGH);
			digitalWrite(s->p3,HIGH);
			digitalWrite(s->p4,LOW);
			break;
			case 2:
			digitalWrite(s->p1,LOW);
			digitalWrite(s->p2,HIGH);
			digitalWrite(s->p3,LOW);
			digitalWrite(s->p4,HIGH);
			break;
			case 3:
			digitalWrite(s->p1,HIGH);
			digitalWrite(s->p2,LOW);
			digitalWrite(s->p3,LOW);
			digitalWrite(s->p4,HIGH);
			break;
		}
	}
}
void stepper_step(stepper_t *s, uint8_t steps_to_move)
{
	if(!s) return;
	uint8_t steps_left = steps_to_move > 0 ? steps_to_move : -steps_to_move;
	if(steps_to_move > 0) s->direction = 1;
	if(steps_to_move < 0) s->direction = 0;
	while(steps_left > 0)
	{
		if(millis() - s->last_step_time > s->step_delay)
		{
			s->last_step_time = millis();
			if(s->direction)
			{
				if(s->step_number++ == 4) s->step_number = 0;
			}
			else
			{
				if(s->step_number == 0)
				{
					s->step_number = 4;
				}
				s->step_number--;
			}
			steps_left--;
			stepper_stepMotor(s,s->step_number);
		}
	}
}
