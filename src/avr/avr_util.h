#ifndef AVR_UTIL_H
#define AVR_UTIL_H

#include "stdint.h"

#define OUTPUT	1
#define INPUT	0
#define HIGH 	1
#define LOW	0
void digitalWrite(uint8_t,uint8_t);
void pinMode(uint8_t,uint8_t);
void timer_init();
uint64_t millis();
#endif
