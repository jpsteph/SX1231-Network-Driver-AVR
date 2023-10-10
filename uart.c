/*
 * CFile1.c
 *
 * Created: 12/19/2022 10:29:13 PM
 *  Author: jpsteph
 */ 

#include <avr/io.h>
#include "uart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define LF 10
#define CR 13

void USART_Init( uint16_t baudrate)
{
	//calculating baudrate
	uint16_t baudRegVal = ((F_CPU)/(baudrate*16UL)-1);
	
	/* Set baud rate */
	UBRR1H = (unsigned char) (baudRegVal>>8);
	UBRR1L = (unsigned char) baudRegVal;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) )
	;
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

void USART_Transmit_String(char * s)
{
	while(*s)
	{
		USART_Transmit(*s);
		s++;
	}
	//NL
	USART_Transmit( LF );
	USART_Transmit( CR );
}

void USART_Transmit_Int8(uint8_t intnum)
{
	uint8_t bufsiz = 100;
	char txbuffer[bufsiz];
	
	sprintf(txbuffer, "%d", intnum);
	USART_Transmit_String(txbuffer);
}

void USART_Transmit_Int16(uint16_t intnum)
{
	uint8_t bufsiz = 100;
	char txbuffer[bufsiz];
	
	sprintf(txbuffer, "%d", intnum);
	USART_Transmit_String(txbuffer);
}

void USART_Transmit_Int32(uint32_t intnum)
{
	uint8_t bufsiz = 100;
	char txbuffer[bufsiz];
	
	sprintf(txbuffer, "%ld", intnum);
	USART_Transmit_String(txbuffer);
}

void USART_Transmit_Float(float intnum) 
{
	uint8_t bufsiz = 100;
	char txbuffer[bufsiz];
	
	dtostrf(intnum, 8, 4, txbuffer);
	USART_Transmit_String(txbuffer);
}

void USART_Transmit_String2(char * s, int size)
{
	int i = 0;
	while(i < size - 1)
	{
		USART_Transmit(s[i]);
		i++;
	}
	//NL
	USART_Transmit( LF );
	USART_Transmit(CR );
}



unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	return UDR1;
}

int USART_Receive_String(char* str)
{
	int i = 0;
	unsigned char s = 0;
	
	while(s != CR)
	{
		s = USART_Receive();
		USART_Transmit(s);
		str[i] = s;
		i++;
	}
	return i;
	
}


int USART_cmd(char* str, char* cmd, int size)
{
	int flag = 0;
	
	for(int j = 0; j < size - 1; j++ )
	{
		if(str[j] == cmd[j])
		{
			flag = 1;
			//USART_Transmit_String("Flag Found!");
		}
		else flag = 0;
		
		if(!flag) break;
	}
	return flag;
	
}