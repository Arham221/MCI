#include "adc.h"

#define BUF_SIZE 1024U                    /* power of two. 1024 samples / 10 kHz = 102 ms of cushion */

static volatile uint16_t buf[BUF_SIZE];
static volatile uint16_t head = 0;        /* written only by the interrupt */
static volatile uint16_t tail = 0;        /* written only by main()        */

volatile uint32_t adc_sample_count  = 0;
volatile uint32_t adc_overrun_count = 0;
volatile uint32_t adc_drop_count    = 0;

static void delay_loop(volatile uint32_t n) { while (n--) { } }

void ADC1_Init(void)
{
    /* ---- pin PA0 ---- */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                 /* AHBENR.IOPAEN = 1: clock for GPIOA */
    GPIOA->MODER |= (3U << 0);                         /* MODER0[1:0] = 11: PA0 analog mode */
    GPIOA->PUPDR &= ~(3U << 0);                        /* PUPDR0 = 00: no pull-up/down */

    /* ---- ADC clock ---- */
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;                 /* AHBENR.ADC12EN = 1: clock for ADC1/2 */
    ADC1_2_COMMON->CCR |= (1U << 16);                  /* CCR.CKMODE[1:0] = 01: ADC clock = HCLK/1 (8 MHz) */

    /* ---- voltage regulator on ---- */
    ADC1->CR &= ~ADC_CR_ADVREGEN;                      /* CR.ADVREGEN[1:0] = 00 (intermediate state, required) */
    ADC1->CR |= ADC_CR_ADVREGEN_0;                     /* CR.ADVREGEN = 01: regulator enabled */
    delay_loop(2000);                                  /* wait > 10 us for regulator start-up */

    /* ---- calibration (single-ended) ---- */
    ADC1->CR &= ~ADC_CR_ADCALDIF;                      /* CR.ADCALDIF = 0: single-ended calibration */
    ADC1->CR |= ADC_CR_ADCAL;                          /* CR.ADCAL = 1: start calibration */
    while (ADC1->CR & ADC_CR_ADCAL) { }                /* hardware clears ADCAL when finished */
    delay_loop(100);

    /* ---- configuration (ADC must still be disabled) ---- */
    ADC1->CFGR &= ~ADC_CFGR_RES;                       /* CFGR.RES[1:0] = 00: 12-bit resolution */
    ADC1->CFGR &= ~ADC_CFGR_ALIGN;                     /* CFGR.ALIGN = 0: right-aligned result */
    ADC1->CFGR &= ~ADC_CFGR_CONT;                      /* CFGR.CONT = 0: one conversion per trigger */
    ADC1->CFGR &= ~ADC_CFGR_EXTSEL;                    /* clear EXTSEL[3:0] */
    ADC1->CFGR |= (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_0); /* CFGR.EXTSEL = 1101: trigger = TIM6_TRGO */
    ADC1->CFGR &= ~ADC_CFGR_EXTEN;                     /* clear EXTEN[1:0] */
    ADC1->CFGR |= ADC_CFGR_EXTEN_0;                    /* CFGR.EXTEN = 01: trigger on rising edge */

    ADC1->SMPR1 &= ~(7U << 3);                         /* SMPR1.SMP1[2:0] clear (channel 1) */
    ADC1->SMPR1 |=  (4U << 3);                         /* SMP1 = 100: 19.5 ADC cycles sampling time */

    ADC1->SQR1 &= ~(0xFU);                             /* SQR1.L[3:0] = 0000: sequence length = 1 conversion */
    ADC1->SQR1 &= ~(0x1FU << 6);                       /* SQR1.SQ1[4:0] clear */
    ADC1->SQR1 |=  (1U << 6);                          /* SQR1.SQ1 = 1: first conversion is channel 1 (PA0) */

    /* ---- interrupt ---- */
    ADC1->IER |= ADC_IER_EOCIE;                        /* IER.EOCIE = 1: interrupt at end of conversion */
    ADC1->IER |= ADC_IER_OVRIE;                        /* IER.OVRIE = 1: interrupt on overrun (error counter) */
    NVIC_SetPriority(ADC1_2_IRQn, 1);
    NVIC_EnableIRQ(ADC1_2_IRQn);                       /* NVIC: allow ADC1_2 interrupt */

    /* ---- enable ADC ---- */
    ADC1->ISR |= ADC_ISR_ADRDY;                        /* ISR.ADRDY = 1 (write 1 clears the flag) */
    ADC1->CR |= ADC_CR_ADEN;                           /* CR.ADEN = 1: enable ADC */
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }           /* wait until ADC is ready */
}

void ADC1_Start(void)
{
    ADC1->CR |= ADC_CR_ADSTART;                        /* CR.ADSTART = 1: arm; waits for hardware trigger */
}

void ADC1_2_IRQHandler(void)
{
    if (ADC1->ISR & ADC_ISR_OVR) {
        ADC1->ISR = ADC_ISR_OVR;                       /* ISR.OVR: write 1 to clear */
        adc_overrun_count++;
    }
    if (ADC1->ISR & ADC_ISR_EOC) {
        uint16_t v = (uint16_t)ADC1->DR;               /* reading DR returns result AND clears EOC */
        adc_sample_count++;

        uint16_t next = (uint16_t)((head + 1U) & (BUF_SIZE - 1U));
        if (next == tail) {
            adc_drop_count++;                          /* buffer full: count the loss, never hide it */
        } else {
            buf[head] = v;
            head = next;
        }
    }
}

uint8_t ADC_Read(uint16_t *sample)
{
    if (head == tail) return 0;                        /* empty */
    *sample = buf[tail];
    tail = (uint16_t)((tail + 1U) & (BUF_SIZE - 1U));
    return 1;
}
