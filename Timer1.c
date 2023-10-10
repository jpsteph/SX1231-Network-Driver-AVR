/*
 * Timer1.c
 *
 * Created: 2/28/2023 9:27:47 PM
 *  Author: jpsteph
 */ 


#include <avr/interrupt.h>
#include "uart.h"

volatile uint16_t tic_count1 = 0;
volatile uint8_t timer1_done = 0;
volatile uint16_t timer1_tic_thres = 0;

void Timer1_init(void) 
{
	/* Timer clock = I/O clock / 1024 */
	TCCR1B = (1<<CS12)|(1<<CS10);
	/* Clear overflow flag */
	TIFR1 = 1<<TOV1;
	/* Enable Overflow Interrupt */
	TIMSK1 = 1<<TOIE1;
	
}

void Timer1_disable(void) 
{
	TCCR1B = (0<<CS12)|(0<<CS10);
}

void Timer1_enable(void) 
{
	TCCR1B = (1<<CS12)|(1<<CS10);
}


void Reset_Timer1_Status(void) 
{
	tic_count1 = 0;
	timer1_tic_thres = 0;
	timer1_done = 0;
	
}

void Timer1_Handle(uint16_t tic_thres)
{
	timer1_tic_thres = tic_thres;
	Timer1_enable();
	while(timer1_done == 0);
	Timer1_disable();
	Reset_Timer1_Status();
}

//interrupt fires roughly every 8 seconds
ISR(TIMER1_OVF_vect)
{
	tic_count1++;
	//USART_Transmit_String("TIMER 1 TIC:");
	USART_Transmit_Int16(tic_count1);  //debug
	if (tic_count1 > timer1_tic_thres) {
		timer1_done = 1;
		//USART_Transmit_String("TIMER IS DONE");

	}
}