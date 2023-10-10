/*
 * LED.c
 *
 * Created: 2/25/2023 12:50:33 PM
 *  Author: jpsteph
 */ 
#include <avr/io.h>
#include <util/delay.h>

#define LED 7

void LEDINIT(void) {
	DDRC |= (1 << LED);
}

void LEDBLINK(void) {
	PORTC |= (1 << LED);
	_delay_ms(500);
	PORTC &= ~(1 << LED);
	_delay_ms(500);
}

void LEDBLINKSHORT(void) {
	PORTC |= (1 << LED);
	_delay_ms(50);
	PORTC &= ~(1 << LED);
	_delay_ms(50);
}

void LEDBLINKLONG(void) {
	PORTC |= (1 << LED);
	_delay_ms(1000);
	PORTC &= ~(1 << LED);
	_delay_ms(1000);
}