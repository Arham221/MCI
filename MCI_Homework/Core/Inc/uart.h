#ifndef UART_H
#define UART_H
#include "stm32f303xc.h"
#include <stdint.h>

void UART1_Init(uint32_t baud);        /* USART1 TX on PA9, 8N1 */
void UART1_SendByte(uint8_t b);        /* polling send */

#endif
