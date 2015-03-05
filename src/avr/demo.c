#include "avr_util.h"
#include "robot.h"
#include "stepper.h"
#include "stdint.h"
#include "avr/io.h"
#define EVAL_TIMES 5
#define NUM_MOTORS 3


stepper_t motors[NUM_MOTORS] =	{
				stepper2(PINB0,PINB1),
				stepper2(PINB2,PINB3),
				stepper2(PINB4,PINB5)
				};
double joints[NUM_MOTORS] = {6,8,5};
double angles[NUM_MOTORS] = {0,0,0};

void move(int index, double angle)
{
	stepper_step(motors+index,4*angle/TAU);
}
uint8_t isCommanded(uint8_t *x, uint8_t *y)
{
	//*x = ???;
	//*y = ???;
	return (PINB & (1<<PINB6));
}
int main()
{
	pinMode(PINB6,INPUT);//the command trigger
	timer_init();
	uint8_t x,y,i;
	for(i = 0; i < NUM_MOTORS; i++)
	{
		stepper_setSpeed(motors+i,M_RPM);
		stepper_motorStep(motors+i,0);//set at 0 radians
	}
	for(;;)
	{
		if(isCommanded(&x,&y))
		{
			moveTowards(NUM_MOTORS,joints,angles,EVAL_TIMES,x,y,move);
		}
	}
	return 0;
}
