#ifndef ADC_H
#define ADC_H
#include "stm32f303xc.h"
#include <stdint.h>

void     ADC1_Init(void);                 /* PA0 / ADC1_IN1, 12-bit, TIM6 triggered, EOC interrupt */
void     ADC1_Start(void);                /* arm ADC: it now converts on every timer trigger */
uint8_t  ADC_Read(uint16_t *sample);      /* pop one sample from buffer: 1 = got one, 0 = empty */

extern volatile uint32_t adc_sample_count;   /* every sample ever converted */
extern volatile uint32_t adc_overrun_count;  /* ADC overrun (ISR too slow)  -> must stay 0 */
extern volatile uint32_t adc_drop_count;     /* buffer full (main too slow) -> must stay 0 */

#endif
