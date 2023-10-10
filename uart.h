/*
 * uart.h
 *
 * Created: 12/19/2022 10:30:17 PM
 *  Author: jpsteph
 */ 


#ifndef UART_H_
#define UART_H_

void USART_Init( uint16_t baud);

void USART_Transmit( unsigned char data );

void USART_Transmit_String(char * s);

void USART_Transmit_Int8(uint8_t intnum);

void USART_Transmit_Int16(uint16_t intnum);

void USART_Transmit_Int32(uint32_t intnum);

void USART_Transmit_Float(float intnum);

void USART_Transmit_String2(char * s, int size);

unsigned char USART_Receive( void );

int USART_Receive_String( char * s);

int USART_cmd(char* str, char* cmd, int size);

#endif /* UART_H_ */