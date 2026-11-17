#include "timer.h"

/* System clock = 8 MHz (HSE from ST-LINK, see Clock_UseHSE in main.c).
 * f_trigger = f_clk / ((PSC+1) * (ARR+1)) = 8,000,000 / (8 * 100) = 10,000 Hz */
void Timer6_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;   /* APB1ENR.TIM6EN = 1: give clock to TIM6 */

    TIM6->PSC = 7;                        /* PSC = 7  -> counter clock = 8 MHz / 8 = 1 MHz */
    TIM6->ARR = 99;                       /* ARR = 99 -> update event every 100 ticks = 100 us = 10 kHz */

    TIM6->CR2 &= ~TIM_CR2_MMS;            /* CR2.MMS[2:0] clear */
    TIM6->CR2 |= TIM_CR2_MMS_1;           /* CR2.MMS = 010 -> TRGO is sent on every UPDATE event */

    TIM6->EGR = TIM_EGR_UG;               /* EGR.UG = 1: load PSC/ARR into the working registers now */
}

void Timer6_Start(void)
{
    TIM6->CR1 |= TIM_CR1_CEN;             /* CR1.CEN = 1: counter runs */
}
