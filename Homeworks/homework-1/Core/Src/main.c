/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Homework 1 Part 1B (Interrupt-Driven)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//PART 1B/A
// #define LED_PIN  8U   // PE8 (LD4 Blue LED on STM32F3 Discovery)
// #define BTN_PIN  0U   // PA0 (USER Push Button)
//END
//PART 2
#define LED_PIN  8U   // PE8 (LD4 Blue LED) - Part 1A/1B
#define BTN_PIN  0U   // PA0 (USER Push Button)

// Part 2
#define LED_R_PIN 9U    // PE9  LD3 red
#define LED_G_PIN 11U   // PE11 LD7 green
#define LED_B_PIN 12U   // PE12 LD9 blue
#define W_DELAY_MS    150U   // white on/off
#define RGB_DELAY_MS  400U   // per color
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
//PART 2
volatile uint8_t g_flash_LED = 0;   // 1 = SW1 held (flash white), 0 = RGB cycle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//PART 2
static void Control_RGB_LEDs(int r, int g, int b)
{
    GPIOE->BSRR = r ? (1U << LED_R_PIN) : (1U << (LED_R_PIN + 16U));
    GPIOE->BSRR = g ? (1U << LED_G_PIN) : (1U << (LED_G_PIN + 16U));
    GPIOE->BSRR = b ? (1U << LED_B_PIN) : (1U << (LED_B_PIN + 16U));
}

// Timer: replaces Delay(), TIM2 in one-pulse (one-shot) mode
static void TIM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 48000U - 1U;   // 48 MHz / 48000 = 1 kHz -> 1 ms/count
    TIM2->CR1 = TIM_CR1_OPM;   // one-pulse: stops itself at expiry
    TIM2->EGR = TIM_EGR_UG;    // load PSC now
    TIM2->SR  = 0;             // clear flag UG just set
}
static void Start_TIM(uint32_t ms)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->ARR = ms - 1U;
    TIM2->CNT = 0;
    TIM2->SR  = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}
static uint32_t TIM_Expired(void) { return (TIM2->SR & TIM_SR_UIF) ? 1U : 0U; }
static void Stop_TIM(void)        { TIM2->CR1 &= ~TIM_CR1_CEN; TIM2->SR = 0; }

// Task 1: white flash FSM (runs only while SW1 is held)
static void Task_Flash_FSM_Timer(void)
{
    enum State { ST_WHITE, ST_WHITE_WAIT, ST_BLACK, ST_BLACK_WAIT };
    static enum State next_state = ST_WHITE;

    if (g_flash_LED == 1) {
        switch (next_state) {
        case ST_WHITE:
            Control_RGB_LEDs(1, 1, 1);
            Start_TIM(W_DELAY_MS);
            next_state = ST_WHITE_WAIT;
            break;
        case ST_WHITE_WAIT:
            if (TIM_Expired()) { Stop_TIM(); next_state = ST_BLACK; }
            break;
        case ST_BLACK:
            Control_RGB_LEDs(0, 0, 0);
            Start_TIM(W_DELAY_MS);
            next_state = ST_BLACK_WAIT;
            break;
        case ST_BLACK_WAIT:
            if (TIM_Expired()) { Stop_TIM(); next_state = ST_WHITE; }
            break;
        default:
            next_state = ST_WHITE;
            break;
        }
    } else {
        next_state = ST_WHITE;   // reset cleanly for next time SW1 is held
    }
}

// Task 2: RGB cycle FSM (runs only while SW1 is NOT held)
static void Task_RGB_FSM_Timer(void)
{
    enum State { ST_RED, ST_RED_WAIT, ST_GREEN, ST_GREEN_WAIT,
                 ST_BLUE, ST_BLUE_WAIT };
    static enum State next_state = ST_RED;

    if (g_flash_LED == 0) {
        switch (next_state) {
        case ST_RED:
            Control_RGB_LEDs(1, 0, 0);
            Start_TIM(RGB_DELAY_MS);
            next_state = ST_RED_WAIT;
            break;
        case ST_RED_WAIT:
            if (TIM_Expired()) { Stop_TIM(); next_state = ST_GREEN; }
            break;
        case ST_GREEN:
            Control_RGB_LEDs(0, 1, 0);
            Start_TIM(RGB_DELAY_MS);
            next_state = ST_GREEN_WAIT;
            break;
        case ST_GREEN_WAIT:
            if (TIM_Expired()) { Stop_TIM(); next_state = ST_BLUE; }
            break;
        case ST_BLUE:
            Control_RGB_LEDs(0, 0, 1);
            Start_TIM(RGB_DELAY_MS);
            next_state = ST_BLUE_WAIT;
            break;
        case ST_BLUE_WAIT:
            if (TIM_Expired()) { Stop_TIM(); next_state = ST_RED; }
            break;
        default:
            next_state = ST_RED;
            break;
        }
    } else {
        next_state = ST_RED;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize CubeMX generated peripherals (RTC, SPI, I2C, USB) */
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  /* =========================================================================
   * PART 1A: CMSIS Direct Register Configuration (Polling) - [COMMENTED OUT]
   * =========================================================================
  // 1. Enable AHB peripheral clock for GPIOA (Button) and GPIOE (LEDs)
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN;

  // 2. Configure PE8 as General Purpose Output (Bits 17:16 = 01)
  GPIOE->MODER &= ~(3U << (LED_PIN * 2)); // Clear mode bits
  GPIOE->MODER |=  (1U << (LED_PIN * 2)); // Set as Output (01)
  GPIOE->OTYPER &= ~(1U << LED_PIN);       // Push-pull output
  GPIOE->OSPEEDR &= ~(3U << (LED_PIN * 2));// Low speed

  // 3. Configure PA0 as Digital Input (Bits 1:0 = 00)
  GPIOA->MODER &= ~(3U << (BTN_PIN * 2));  // Set mode to Input (00)
  
  // 4. Set Pull-down resistor on PA0
  GPIOA->PUPDR &= ~(3U << (BTN_PIN * 2));  // Clear pull configuration
  GPIOA->PUPDR |=  (2U << (BTN_PIN * 2));  // Set pull-down (10)
  ========================================================================= */

  // =========================================================================
  // PART 1B: CMSIS Direct Register Configuration (Interrupt-Driven)
  // =========================================================================

  // 1. Enable AHB clocks for GPIOA, GPIOE and APB2 clock for SYSCFG
  // RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN;
  // RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // // 2. Configure PE8 as General Purpose Output
  // GPIOE->MODER &= ~(3U << (LED_PIN * 2)); // Clear mode bits
  // GPIOE->MODER |=  (1U << (LED_PIN * 2)); // Set mode to Output (01)
  // GPIOE->OTYPER &= ~(1U << LED_PIN);       // Push-pull output
  // GPIOE->OSPEEDR &= ~(3U << (LED_PIN * 2));// Low speed

  // // 3. Configure PA0 as Digital Input with Pull-Down
  // GPIOA->MODER &= ~(3U << (BTN_PIN * 2));  // Input mode (00)
  // GPIOA->PUPDR &= ~(3U << (BTN_PIN * 2));  // Clear pull configuration
  // GPIOA->PUPDR |=  (2U << (BTN_PIN * 2));  // Set pull-down (10)

  // // 4. Connect EXTI Line 0 to Port A (PA0) via SYSCFG_EXTICR1
  // SYSCFG->EXTICR[0] &= ~(0xFU << 0);       // EXTI0 mapped to PA0

  // // 5. Configure EXTI Line 0 for Rising Edge Triggering (Button press)
  // EXTI->IMR  |= EXTI_IMR_MR0;              // Unmask EXTI Line 0
  // EXTI->RTSR |= EXTI_RTSR_TR0;             // Enable rising edge trigger
  // EXTI->FTSR &= ~EXTI_FTSR_TR0;            // Disable falling edge trigger

  // // 6. Enable EXTI Line 0 Interrupt in NVIC
  // NVIC_SetPriority(EXTI0_IRQn, 2);
  // NVIC_EnableIRQ(EXTI0_IRQn);
  // =========================================================================
  // PART 2: Timer + FSM (both edges of SW1)
  // =========================================================================

  // 1. Clocks: GPIOA (button), GPIOE (LEDs, already includes PE8), SYSCFG, TIM2
  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // 2. Configure PE9, PE11, PE12 as outputs (red, green, blue)
  {
      uint32_t pins[3] = { LED_R_PIN, LED_G_PIN, LED_B_PIN };
      for (int i = 0; i < 3; i++) {
          GPIOE->MODER &= ~(3U << (pins[i] * 2));
          GPIOE->MODER |=  (1U << (pins[i] * 2));
      }
  }

  // 3. PA0 input, pull-down (same as Part 1B)
  GPIOA->MODER &= ~(3U << (BTN_PIN * 2));
  GPIOA->PUPDR &= ~(3U << (BTN_PIN * 2));
  GPIOA->PUPDR |=  (2U << (BTN_PIN * 2));

  // 4. EXTI0 <- PA0, BOTH edges this time (press AND release matter)
  SYSCFG->EXTICR[0] &= ~(0xFU << 0);
  EXTI->IMR  |= EXTI_IMR_MR0;
  EXTI->RTSR |= EXTI_RTSR_TR0;   // press
  EXTI->FTSR |= EXTI_FTSR_TR0;   // release
  NVIC_SetPriority(EXTI0_IRQn, 2);
  NVIC_EnableIRQ(EXTI0_IRQn);

  // 5. Timer init, and set initial flag from the button's current state
  TIM_Init();
  g_flash_LED = (uint8_t)((GPIOA->IDR >> BTN_PIN) & 1U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* =========================================================================
     * PART 1A: Continuous Polling Loop - [COMMENTED OUT]
     * =========================================================================
    if ((GPIOA->IDR >> BTN_PIN) & 1U)
    {
      // Button Pressed (HIGH): Turn ON PE8 LED using Bit Set Register
      GPIOE->BSRR = (1U << LED_PIN);
    }
    else
    {
      // Button Released (LOW): Turn OFF PE8 LED using Bit Reset Register
      GPIOE->BSRR = (1U << (LED_PIN + 16U));
    }
    ========================================================================= */

    // =========================================================================
    // PART 1B: Main application loop remains idle/non-blocking
    // =========================================================================
    __NOP();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Task_Flash_FSM_Timer();
    Task_RGB_FSM_Timer();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_RTC_Init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// // PART 1B
// /**
//   * @brief  EXTI Line 0 Interrupt Handler for PA0 Button Press (Part 1B)
//   */
// void EXTI0_IRQHandler(void)
// {
//   // Check if EXTI Line 0 pending flag is set
//   if (EXTI->PR & EXTI_PR_PR0)
//   {
//     // Clear pending bit by writing '1' to bit 0
//     EXTI->PR = EXTI_PR_PR0;

//     // Toggle PE8 LED using Output Data Register (ODR)
//     GPIOE->ODR ^= (1U << LED_PIN);
//   }
// } END
// PART 2
void EXTI0_IRQHandler(void)
{
  if (EXTI->PR & EXTI_PR_PR0)
  {
    EXTI->PR = EXTI_PR_PR0;                              // acknowledge
    g_flash_LED = (uint8_t)((GPIOA->IDR >> BTN_PIN) & 1U); // read current level
  }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}