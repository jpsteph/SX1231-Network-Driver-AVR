/*
 * RFM69.c
 *
 * Created: 1/19/2023 6:32:27 PM
 * Author : jpsteph
 */ 


#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "spi.h"
#include "RFM69.h"
#include "RFM69registers.h"
#include "uart.h"
#include "LED.h"
#include <stdio.h>
#include <avr/sleep.h>

//#define GATEWAY
#define NODE

//for GATEWAY
#ifdef GATEWAY
#define NETWORKID 33
#define NODEID    3
#define TONODEID  4
#endif

//for RX

#ifdef NODE
#define NETWORKID 33
#define NODEID     4
#define TONODEID 3
#endif

#define RST 5 //reset pin for rfm69

int main(void)
{
	
	char rxbuff[66];
	rxbuff[0] = '\0';
	//char wakeupcmd[] = "WAKEUP";
	char sleepcmd[] = "SLEEP";
	char sleepACK[] = "GTSLEEP";
	uint8_t flag = 0;
	
	#ifdef NODE
	char tempbuff[30];
	#endif
	
	 DDRB |= (1 << RST);
	 PORTB &= ~(1 << RST); //need to keep reset pin for rfm69 low to function
	
	 DDRB |= (1 << 0); //need to keep ATMEGA32U4 native SS pin high for SPI to function
	 PORTB |= (1 <<0);
	
	 LEDINIT();
	 LEDBLINKSHORT();
	 
	 USART_Init(9600);
	 USART_Transmit_String("USART INIT");
	 
	 rfm69_init(433, NODEID, NETWORKID);
	 
	 setHighPower(1); 
	 setPowerLevel(30); 
	 encrypt(NULL); 
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();

	
	 while (1)
	 {
		#ifdef NODE
			sprintf(tempbuff, "TEMP %d", readTemperature(5));
			//USART_Transmit_String(tempbuff);
			sendFrame(TONODEID, "TEST"); 
			_delay_ms(500);
			receiveEvent(rxbuff);
		
			if(rxbuff[0] != '\0') {
			sleep_disable();
			USART_Transmit_String2(rxbuff, DATALEN+1);
			
			flag = USART_cmd(rxbuff, sleepcmd, 5);
			if(flag) 
			{
				USART_Transmit_String("GOING TO SLEEP");
				sendFrame(TONODEID, sleepACK);
				sleep_enable();
				sleep(5);
			}
			rxbuff[0] = '\0';
			DATA[0] = '\0';
			sleep_enable();	
			}
		#endif 
		 
		#ifdef GATEWAY
			receiveEvent(rxbuff);
			 
			if(rxbuff[0] != '\0') {
				sleep_disable();
				USART_Transmit_String2(rxbuff, DATALEN+1);
				 
				flag = USART_cmd(rxbuff, "TEMP", 4);
				if(flag)
				{
					USART_Transmit_String("SENDING SLEEP CMD");
					sendFrame(TONODEID, sleepcmd);
				}
				flag = USART_cmd(rxbuff, sleepACK, 7);
				if(flag)
				{
					USART_Transmit_String("SLEEP ACK RECEIVED");
					sleep_enable();
					sleep(5);
				}
				 
				rxbuff[0] = '\0';
				DATA[0] = '\0';
				sleep_enable();
			}
		#endif
		//LEDBLINK(); 
	}
}

