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

int main(void)
{
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
