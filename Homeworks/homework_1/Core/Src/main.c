/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * 
  *  NAME: SYED MUHAMMAD HAIDER ZAIDI | DANISH ALI KHAN
  *  STUDENT ID: mz10270 | dk10402
  *  COURSE: MCI | SEMESTER: 5
  *  HOMEWORK: 1
  ******************************************************************************
  */

/* USER CODE END Header */

#include "stm32f3xx.h" // Replaced main.h with CMSIS device header

// Global variable updated by your SW1 interrupt (1 = Flash, 0 = RGB)
volatile uint8_t g_flash_LED = 0; 

// --- Function Prototypes ---
void Interrupt_Init(void);
void Init_GPIO_RGB(void);
void Control_RGB_LEDs(uint8_t red, uint8_t green, uint8_t blue);
void Start_Timer(uint16_t delay_ms);
uint8_t Timer_Expired(void);
void Task_RGB_FSM(void);
void Task_Flash_FSM(void);

// --- Interrupt Service Routine ---
void EXTI0_IRQHandler(void) {
    // Check if the EXTI0 interrupt pending flag is set
    if (EXTI->PR & EXTI_PR_PR0) {
        // Acknowledge (clear) the interrupt by writing 1 to the Pending Register
        EXTI->PR = EXTI_PR_PR0; 

        // Read the switch state from Port A Input Data Register (IDR)
        if (GPIOA->IDR & GPIO_IDR_0) {
            // Switch is pressed: Tell the FSM to flash white
            g_flash_LED = 1; 
        } else {
            // Switch is released: Tell the FSM to run RGB sequence
            g_flash_LED = 0; 
        }
    }
}

// --- Hardware Timer Functions (Non-Blocking) ---
void Start_Timer(uint16_t delay_ms) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM6->CR1 &= ~TIM_CR1_CEN;     // stop timer in case it was running
    TIM6->PSC  = 8000 - 1;         // 8 MHz / 8000 = 1 kHz (1 ms tick)
    TIM6->ARR  = delay_ms - 1;
    TIM6->CNT  = 0;                // restart from zero (this was missing)
    TIM6->EGR  = TIM_EGR_UG;       // force update so PSC/ARR are loaded
    TIM6->SR  &= ~TIM_SR_UIF;      // UG sets UIF, so clear it
    TIM6->CR1 |= TIM_CR1_CEN;      // start
}

uint8_t Timer_Expired(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF;   
        TIM6->CR1 &= ~TIM_CR1_CEN; 
        return 1;                  
    }
    return 0;                      
}

// --- Helper function to control RGB LEDs ---
void Control_RGB_LEDs(uint8_t red, uint8_t green, uint8_t blue) {
    if (red)   { GPIOE->BSRR = GPIO_BSRR_BS_9; }  else { GPIOE->BSRR = GPIO_BSRR_BR_9; }
    if (green) { GPIOE->BSRR = GPIO_BSRR_BS_10; } else { GPIOE->BSRR = GPIO_BSRR_BR_10; }
    if (blue)  { GPIOE->BSRR = GPIO_BSRR_BS_11; } else { GPIOE->BSRR = GPIO_BSRR_BR_11; }
}

// --- Hardware Initialization ---
void Init_GPIO_RGB(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    GPIOE->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
    GPIOE->MODER |= (GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);
}

void Interrupt_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    
    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->FTSR |= EXTI_FTSR_TR0;
    EXTI->RTSR |= EXTI_RTSR_TR0;

    NVIC_EnableIRQ(EXTI0_IRQn);
}

// --- FSM Task Implementations ---
void Task_RGB_FSM(void) {
    enum State { ST_RED, ST_RED_WAIT, ST_GREEN, ST_GREEN_WAIT, ST_BLUE, ST_BLUE_WAIT };
    static enum State current_state = ST_RED;
    static uint8_t was_interrupted = 0; // Flag to track if the sequence was paused
    
    // Only run the RGB sequence if the switch is released (g_flash_LED == 0)
    if (g_flash_LED == 0) { 
        
        // If we just finished flashing, we need to re-apply the color we paused on
        // and restart the 500ms timer so the sequence resumes cleanly.
        if (was_interrupted) {
            if (current_state == ST_RED_WAIT) current_state = ST_RED;
            else if (current_state == ST_GREEN_WAIT) current_state = ST_GREEN;
            else if (current_state == ST_BLUE_WAIT) current_state = ST_BLUE;
            was_interrupted = 0; // Clear the flag
        }

        switch (current_state) {
            case ST_RED:
                Control_RGB_LEDs(1, 0, 0); 
                Start_Timer(500);          
                current_state = ST_RED_WAIT;
                break;
                
            case ST_RED_WAIT:
                if (Timer_Expired()) { current_state = ST_GREEN; }
                break;
                
            case ST_GREEN:
                Control_RGB_LEDs(0, 1, 0); 
                Start_Timer(500);          
                current_state = ST_GREEN_WAIT;
                break;
                
            case ST_GREEN_WAIT:
                if (Timer_Expired()) { current_state = ST_BLUE; }
                break;
                
            case ST_BLUE:
                Control_RGB_LEDs(0, 0, 1); 
                Start_Timer(500);          
                current_state = ST_BLUE_WAIT;
                break;
                
            case ST_BLUE_WAIT:
                if (Timer_Expired()) { current_state = ST_RED; }
                break;
                
            default:
                current_state = ST_RED;
                break;
        }
    } else {
        // The button is held down. Mark the interruption so it resumes later.
        was_interrupted = 1;
    }
}

void Task_Flash_FSM(void) {
    enum State { ST_WHITE, ST_WHITE_WAIT, ST_BLACK, ST_BLACK_WAIT };
    static enum State current_state = ST_WHITE;
    
    // Only run the Flash sequence if the switch is pressed (g_flash_LED == 1)
    if (g_flash_LED == 1) { 
        switch (current_state) {
            case ST_WHITE:
                Control_RGB_LEDs(1, 1, 1); // Action: All ON (White)
                Start_Timer(100);          // Action: Start timer for fast flash (100ms)
                current_state = ST_WHITE_WAIT;
                break;
                
            case ST_WHITE_WAIT:
                if (Timer_Expired()) { current_state = ST_BLACK; }
                break;
                
            case ST_BLACK:
                Control_RGB_LEDs(0, 0, 0); // Action: All OFF
                Start_Timer(100);          // Action: Start timer
                current_state = ST_BLACK_WAIT;
                break;
                
            case ST_BLACK_WAIT:
                if (Timer_Expired()) { current_state = ST_WHITE; }
                break;
                
            default:
                current_state = ST_WHITE;
                break;
        }
    } else {
        // Reset the flash state machine when the button is released so it starts with white next time
        current_state = ST_WHITE;
    }
}

// --- Main Program ---
int main(void)
{
    Init_GPIO_RGB();
    Interrupt_Init();
    g_flash_LED = (GPIOA->IDR & GPIO_IDR_0) ? 1 : 0;   // sync initial state

    while (1)
    {
        Task_RGB_FSM();
        Task_Flash_FSM();
    }
}