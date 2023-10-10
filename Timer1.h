/*
 * Timer1.h
 *
 * Created: 2/28/2023 9:28:09 PM
 *  Author: jpsteph
 */ 


#ifndef TIMER1_H_
#define TIMER1_H_

void Timer1_init(void);

void Timer1_disable(void);

void Timer1_enable(void);

void Timer1_Handle(uint16_t tic_thres);

void Reset_Timer1_Status(void);

#endif /* TIMER1_H_ */