#include "stm32f303xc.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"

#define BAUD 230400U

volatile uint32_t samples_last_second = 0;   /* add to Live Expressions: must read 10000 */
static volatile uint32_t ms = 0;
static uint32_t prev_count = 0;

void SysTick_Handler(void)                   /* fires every 1 ms */
{
    if (++ms >= 1000U) {
        ms = 0;
        samples_last_second = adc_sample_count - prev_count;
        prev_count = adc_sample_count;
    }
}

/* Run from the ST-LINK 8 MHz clock (HSE bypass) instead of the internal HSI (+-1 %).
 * Speed stays 8 MHz, so every timer and baud calculation is unchanged. Falls back to HSI. */
static void Clock_UseHSE(void)
{
    uint32_t t;
    RCC->CR |= RCC_CR_HSEBYP;                            /* CR.HSEBYP = 1: external clock on OSC_IN */
    RCC->CR |= RCC_CR_HSEON;                             /* CR.HSEON = 1: start HSE */
    for (t = 0; t < 200000U && !(RCC->CR & RCC_CR_HSERDY); t++) { }   /* wait CR.HSERDY */
    if (RCC->CR & RCC_CR_HSERDY) {
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;      /* CFGR.SW = 01: SYSCLK = HSE */
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) { }     /* wait CFGR.SWS = HSE */
    } else {
        RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);      /* no HSE: stay on HSI */
    }
}

int main(void)
{
    Clock_UseHSE();
    SysTick_Config(8000U);                   /* 8 MHz / 8000 = 1 kHz tick */

    UART1_Init(BAUD);
    Timer6_Init();
    ADC1_Init();
    ADC1_Start();                            /* ADC armed, waiting for trigger */
    Timer6_Start();                          /* triggers begin: 10 kHz sampling */

    while (1) {
        uint16_t s;
        while (ADC_Read(&s)) {
            UART1_SendByte((uint8_t)(0x80U | (s >> 6)));  /* byte 1: MSB=1 marks "high part", bits 11..6 */
            UART1_SendByte((uint8_t)(s & 0x3FU));         /* byte 2: MSB=0, bits 5..0 */
        }
    }
}
