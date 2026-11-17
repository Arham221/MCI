#include "uart.h"

#define PCLK2_HZ 8000000U              /* USART1 clock = APB2 = 8 MHz (HSE, no PLL) */

void UART1_Init(uint32_t baud)
{
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;                /* clock for GPIOA */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;              /* APB2ENR.USART1EN = 1: clock for USART1 */

    GPIOA->MODER &= ~(3U << 18);                       /* clear MODER9 */
    GPIOA->MODER |=  (2U << 18);                       /* MODER9 = 10: PA9 alternate function */
    GPIOA->AFR[1] &= ~(0xFU << 4);                     /* clear AFRH9 */
    GPIOA->AFR[1] |=  (7U << 4);                       /* AFRH9 = 0111 (AF7): PA9 = USART1_TX */

    USART1->BRR = (PCLK2_HZ + baud / 2U) / baud;       /* BRR = f_clk / baud (rounded), oversampling by 16 */
    USART1->CR1 = USART_CR1_TE;                        /* CR1.TE = 1: transmitter on; M=00 (8 bit), PCE=0 (no parity) */
    USART1->CR2 = 0;                                   /* CR2.STOP = 00: 1 stop bit  -> 8N1 */
    USART1->CR1 |= USART_CR1_UE;                       /* CR1.UE = 1: USART enabled */
}

void UART1_SendByte(uint8_t b)
{
    while (!(USART1->ISR & USART_ISR_TXE)) { }         /* wait until TDR is empty */
    USART1->TDR = b;                                   /* writing TDR starts transmission */
}
