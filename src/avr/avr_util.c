#include "avr_util.h"
#include "avr/io.h"
#include "stdint.h"
volatile uint64_t ms = 0;
void pinMode(uint8_t pin, uint8_t mode)
{
	if(mode) DDRB |= 1 << pin;
	else DDRB &= 1 << pin;
}
void digitalWrite(uint8_t pin, uint8_t mode)
{
	if(mode) PORTB |= 1 << pin;
	else PORTB &= 1 << pin;
}
ISR(TIMER0_0VF_vect)
{
//TCNT0 = 0xF0;
ms++;
}
uint64_t millis()
{
	return ms;
}
void timer_init()
{
	TCCR0 = 0x07;
	TCNT0 = 0xF0;
	TIMSK = 0x01;
}
