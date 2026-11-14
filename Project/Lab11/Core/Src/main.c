/* USER CODE BEGIN Header */
  /**
    ******************************************************************************
    * @file           : main.c
    * @brief          : Main program body
    ******************************************************************************
    * @attention
    *
    * Copyright (c) 2026 STMicroelectronics.
    * All rights reserved.
    *
    * This software is licensed under terms that can be found in the LICENSE file
    * in the root directory of this software component.
    * If no LICENSE file comes with this software, it is provided AS-IS.
    *
    ******************************************************************************
    */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  #include <stdio.h>
  #include <string.h>
  #include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float ax, ay, az;      // g
  float gx, gy, gz;      // deg/s (gyro bias already removed)
} IMU_Data;

typedef struct {
  float kp, ki, kd;
  float integral;        // already multiplied by ki
  float out;
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AXIS_TEST        0        // 1 = print raw sensor values, no motors. Set to 0 afterwards.

#define CONTROL_HZ       200
#define DT               (1.0f / CONTROL_HZ)   // 0.005 s, used by the filter AND the PID
#define COMP_ALPHA       0.98f
#define RAD2DEG          57.2957795f
#define PWM_MAX          999.0f                // = TIM3 ARR
#define MOTOR_DEADZONE   0.0f                  // tune later
#define INTEGRAL_LIMIT   400.0f                // anti-windup
#define FALL_ANGLE_DEG   35.0f                 // motors cut beyond this
#define PRINT_DIV        10                    // 200 Hz / 10 = 20 Hz to UART
#define BALANCE_SIGN     1.0f                  // flip to -1.0f if motors push the wrong way

/* accelerometer (I2C) */
#define ACC_ADDR         0x32
#define ACC_LSB_PER_G    16384.0f              // +-2 g, 16-bit left-justified

/* gyroscope (SPI1, chip select PE3) */
#define GYRO_CS_LOW()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define GYRO_DPS_PER_LSB 0.00875f              // +-245 dps

/* ---- axis choice: set these after the axis test (Step 3) ---- */
#define TILT_ACC_NUM     ay
#define TILT_ACC_DEN     az
#define TILT_ACC_SIGN    -1.0f
#define TILT_GYRO        gy
#define TILT_GYRO_SIGN   1.0f

/* motor pins */
#define R_IN1_PORT GPIOA
#define R_IN1_PIN  GPIO_PIN_1
#define R_IN2_PORT GPIOF
#define R_IN2_PIN  GPIO_PIN_4
#define L_IN1_PORT GPIOC
#define L_IN1_PIN  GPIO_PIN_10
#define L_IN2_PORT GPIOC
#define L_IN2_PIN  GPIO_PIN_9
#define R_INVERT   0                           // set to 1 if a wheel spins the wrong way
#define L_INVERT   0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
IMU_Data g_imu;
PID_t    g_pid = { .kp = 40.0f, .ki = 0.0f, .kd = 1.0f };   // starting guesses (PD is allowed)
float    g_setpoint = 0.0f;                                 // upright angle in degrees
float    g_gx_off = 0, g_gy_off = 0, g_gz_off = 0;          // gyro bias
volatile float    g_acc_angle = 0, g_gyro_rate = 0, g_angle = 0;
volatile uint8_t  g_print_flag = 0;
volatile uint8_t  g_enabled = 0;                            // blue button toggles motors
volatile uint32_t g_isr_us = 0;                             // how long one control run takes
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------- sensors ---------- */
HAL_StatusTypeDef IMU_Init(void)
{
  HAL_StatusTypeDef st = HAL_OK;
  uint8_t v = 0x67;                       // accel: 200 Hz, normal mode, XYZ on
  if (HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, 0x20, 1, &v, 1, 10) != HAL_OK) st = HAL_ERROR;
  v = 0x80;                               // block data update, +-2 g
  if (HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, 0x23, 1, &v, 1, 10) != HAL_OK) st = HAL_ERROR;

  GYRO_CS_HIGH();
  uint8_t tx[2] = { 0x20, 0x4F };         // gyro: 200 Hz, power on, XYZ on
  GYRO_CS_LOW();
  if (HAL_SPI_Transmit(&hspi1, tx, 2, 10) != HAL_OK) st = HAL_ERROR;
  GYRO_CS_HIGH();
  return st;
}

HAL_StatusTypeDef IMU_Read(IMU_Data *d)
{
  uint8_t a[6], g[6];
  if (HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, 0x28 | 0x80, 1, a, 6, 3) != HAL_OK)
    return HAL_ERROR;

  uint8_t cmd = 0x80 | 0x40 | 0x28;       // read + auto-increment + OUT_X_L
  GYRO_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &cmd, 1, 3);
  if (st == HAL_OK) st = HAL_SPI_Receive(&hspi1, g, 6, 3);
  GYRO_CS_HIGH();
  if (st != HAL_OK) return HAL_ERROR;

  d->ax = (int16_t)((a[1] << 8) | a[0]) / ACC_LSB_PER_G;
  d->ay = (int16_t)((a[3] << 8) | a[2]) / ACC_LSB_PER_G;
  d->az = (int16_t)((a[5] << 8) | a[4]) / ACC_LSB_PER_G;
  d->gx = (int16_t)((g[1] << 8) | g[0]) * GYRO_DPS_PER_LSB - g_gx_off;
  d->gy = (int16_t)((g[3] << 8) | g[2]) * GYRO_DPS_PER_LSB - g_gy_off;
  d->gz = (int16_t)((g[5] << 8) | g[4]) * GYRO_DPS_PER_LSB - g_gz_off;
  return HAL_OK;
}

void IMU_CalibrateGyro(uint16_t samples)  // board must be still
{
  float sx = 0, sy = 0, sz = 0;
  uint16_t n = 0;
  g_gx_off = g_gy_off = g_gz_off = 0;
  for (uint16_t i = 0; i < samples; i++) {
    IMU_Data d;
    if (IMU_Read(&d) == HAL_OK) { sx += d.gx; sy += d.gy; sz += d.gz; n++; }
    HAL_Delay(5);
  }
  if (n) { g_gx_off = sx / n; g_gy_off = sy / n; g_gz_off = sz / n; }
}

void IMU_TiltInputs(const IMU_Data *d, float *acc_angle_deg, float *gyro_rate_dps)
{
  *acc_angle_deg = TILT_ACC_SIGN * atan2f(d->TILT_ACC_NUM, d->TILT_ACC_DEN) * RAD2DEG;
  *gyro_rate_dps = TILT_GYRO_SIGN * d->TILT_GYRO;
}

/* angle = 0.98*(angle + gyro*dt) + 0.02*acc   <- the manual's equation */
float IMU_ComplementaryUpdate(float angle, float acc_angle, float gyro_rate)
{
  return COMP_ALPHA * (angle + gyro_rate * DT) + (1.0f - COMP_ALPHA) * acc_angle;
}

/* ---------- PID ---------- */
void PID_Reset(PID_t *c) { c->integral = 0.0f; c->out = 0.0f; }

void PID_Update(PID_t *c, float setpoint, float angle, float gyro_rate)
{
  float error = setpoint - angle;

  c->integral += c->ki * error * DT;                       // I term, with anti-windup
  if (c->integral >  INTEGRAL_LIMIT) c->integral =  INTEGRAL_LIMIT;
  if (c->integral < -INTEGRAL_LIMIT) c->integral = -INTEGRAL_LIMIT;

  // D term: d(error)/dt = -d(angle)/dt = -gyro_rate (setpoint is constant)
  float out = c->kp * error + c->integral + c->kd * (-gyro_rate);

  if (out >  PWM_MAX) out =  PWM_MAX;                      // PWM can't do more
  if (out < -PWM_MAX) out = -PWM_MAX;
  c->out = out;
}

/* ---------- motors ---------- */
static void drive(GPIO_TypeDef *p1, uint16_t pin1, GPIO_TypeDef *p2, uint16_t pin2,
                  uint32_t ch, float speed, int invert)
{
  if (invert) speed = -speed;
  if (speed >  PWM_MAX) speed =  PWM_MAX;
  if (speed < -PWM_MAX) speed = -PWM_MAX;

  if (speed > 0.5f) {                       // forward
    HAL_GPIO_WritePin(p1, pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p2, pin2, GPIO_PIN_RESET);
  } else if (speed < -0.5f) {               // backward
    HAL_GPIO_WritePin(p1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p2, pin2, GPIO_PIN_SET);
    speed = -speed;
  } else {                                  // stop
    HAL_GPIO_WritePin(p1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p2, pin2, GPIO_PIN_RESET);
    speed = 0.0f;
  }
  float duty = 0.0f;
  if (speed > 0.0f)
    duty = MOTOR_DEADZONE + speed * (PWM_MAX - MOTOR_DEADZONE) / PWM_MAX;
  __HAL_TIM_SET_COMPARE(&htim3, ch, (uint32_t)duty);
}

void Motor_SetRight(float s) { drive(R_IN1_PORT, R_IN1_PIN, R_IN2_PORT, R_IN2_PIN, TIM_CHANNEL_1, s, R_INVERT); }
void Motor_SetLeft (float s) { drive(L_IN1_PORT, L_IN1_PIN, L_IN2_PORT, L_IN2_PIN, TIM_CHANNEL_2, s, L_INVERT); }
void Motor_SetBoth(float l, float r) { Motor_SetLeft(l); Motor_SetRight(r); }
void Motor_StopAll(void) { Motor_SetBoth(0, 0); }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
if (IMU_Init() != HAL_OK)
  HAL_UART_Transmit(&huart2, (uint8_t*)"IMU init FAILED\r\n", 17, 100);
HAL_Delay(100);
IMU_CalibrateGyro(400);                 // ~2 s: KEEP THE BOARD STILL

{ float a, r;
  IMU_Read(&g_imu);
  IMU_TiltInputs(&g_imu, &a, &r);
  g_angle = a;                          // start the filter from the accelerometer angle
}

CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // cycle counter, only used to time the ISR
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
Motor_StopAll();

HAL_TIM_Base_Start_IT(&htim6);          // 200 Hz control loop starts here
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    // Test UART transmission before any I2C communication starts
    HAL_UART_Transmit(&huart2, (uint8_t*)"Booting\r\n", 9, HAL_MAX_DELAY);
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint8_t prev_btn = 0;
uint8_t btn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
if (btn && !prev_btn) {                 // blue button toggles the motors
  g_enabled ^= 1;
  HAL_GPIO_WritePin(GPIOE, LD3_Pin, g_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_Delay(200);                       // crude debounce
}
prev_btn = btn;

if (g_print_flag) {
  g_print_flag = 0;
  char buf[100];
#if AXIS_TEST
  int n = snprintf(buf, sizeof buf, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%lu\r\n",
                   g_imu.ax, g_imu.ay, g_imu.az, g_imu.gx, g_imu.gy, g_imu.gz,
                   (unsigned long)g_isr_us);
#else
  // acc angle, gyro rate, FILTERED angle, PID output
  int n = snprintf(buf, sizeof buf, "%.2f,%.2f,%.2f,%.1f\r\n",
                   g_acc_angle, g_gyro_rate, g_angle, g_pid.out);
#endif
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 20);
}
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
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
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Runs exactly every 5 ms, driven by TIM6 hardware: this is the fixed-rate control loop. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t print_div = 0;
  if (htim->Instance != TIM6) return;
  uint32_t t0 = DWT->CYCCNT;

  /* 1. read sensors, 2. complementary filter */
  if (IMU_Read(&g_imu) == HAL_OK) {
    float a, r;
    IMU_TiltInputs(&g_imu, &a, &r);
    g_acc_angle = a;
    g_gyro_rate = r;
    g_angle = IMU_ComplementaryUpdate(g_angle, a, r);
  }

  /* 3. PID on the filtered angle */
  PID_Update(&g_pid, g_setpoint, g_angle, g_gyro_rate);

#if !AXIS_TEST
  /* 4. motors (cut if disabled by the button or the robot has fallen) */
  if (g_enabled && fabsf(g_angle) < FALL_ANGLE_DEG) {
    float cmd = BALANCE_SIGN * g_pid.out;
    Motor_SetBoth(cmd, cmd);
  } else {
    Motor_StopAll();
    PID_Reset(&g_pid);
  }
#endif

  /* 5. slow down UART: 200 Hz / PRINT_DIV */
  if (++print_div >= PRINT_DIV) { print_div = 0; g_print_flag = 1; }

  g_isr_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
