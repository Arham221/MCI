#ifndef TIMER_H
#define TIMER_H
#include "stm32f303xc.h"

void Timer6_Init(void);   /* configure TIM6 to give a 10 kHz TRGO pulse */
void Timer6_Start(void);  /* start counting */

#endif
