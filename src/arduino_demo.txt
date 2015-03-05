//NOTE
//NOTE
//PLEASE READ THIS !!!!!!
//THIS IS NOT A VALID FILE, YOU SHOULD NOT TRY TO COMPILE THIS
//THIS IS JUST "PSEUDOCODE", A SAMPLE IMPLEMENTATION, 
//ALSO FICTIONAL, IT IS JUST AN IDEA

#include "robot.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "stdint.h"

#include "Stepper.h" //arduino library

#define EVAL_TIMES  5 //iterations through this library
#define STEPS_PER_REVOLUTION 4  //number of steps the stepper motor takes in order to complete one revolution
#define NUM_MOTORS 3  //number of motors/arms

uint8_t pins[NUM_MOTORS][4] = {{3,4,5,6},{7,8,9,10},{11,12,13,14}};
Stepper steppers[NUM_MOTORS] = {Stepper(STEPS_PER_REVOLUTION,pins[0][0],pins[0][1],pins[0][2],pins[0][3]),
                                Stepper(STEPS_PER_REVOLUTION,pins[1][0],pins[1][1],pins[1][2],pins[1][3]),
                                Stepper(STEPS_PER_REVOLUTION,pins[2][0],pins[2][1],pins[2][2],pins[2][3])};
double joints[NUM_MOTORS] = {6,8,5};
double angles[NUM_MOTORS] = {0,0,0};

void move(int index, double angle)
{
  steppers[index].step(STEPS_PER_REVOLUTION * angle / TAU);
}

int main(void)
{
  uint8_t x,y,i;
  for(i = 0; i < NUM_MOTORS; i++)
  {
    steppers[i].setSpeed(30);//RPMs
  }
  for(;;)
  {
    if(isCommanded(&x,&y))//if ordered to go to (x,y)
    {
      moveTowards(NUM_MOTORS,joints,angles,EVAL_TIMES,x,y);
    }
  }
  return 0;
}
