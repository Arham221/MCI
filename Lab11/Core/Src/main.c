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

  /* USER CODE END PTD */

  /* Private define ------------------------------------------------------------*/
  /* USER CODE BEGIN PD */

  /* USER CODE END PD */

  /* Private macro -------------------------------------------------------------*/
  /* USER CODE BEGIN PM */

  /* USER CODE END PM */

  /* Private variables ---------------------------------------------------------*/
  I2C_HandleTypeDef hi2c1;

  SPI_HandleTypeDef hspi1;

  UART_HandleTypeDef huart2;

  PCD_HandleTypeDef hpcd_USB_FS;

  /* USER CODE BEGIN PV */
  //task02 begin
  // PID Constants 
  float Kp = 10.0f; 
  float Ki = 0.0f;
  float Kd = 0.0f;

  // PID State Variables
  float setpoint = 0.0f; 
  float integral = 0.0f;
  float prev_error = 0.0f;
  uint32_t prev_time = 0;
  //task02 end
  /* USER CODE END PV */

  /* Private function prototypes -----------------------------------------------*/
  void SystemClock_Config(void);
  static void MX_GPIO_Init(void);
  static void MX_I2C1_Init(void);
  static void MX_SPI1_Init(void);
  static void MX_USB_PCD_Init(void);
  static void MX_USART2_UART_Init(void);
  /* USER CODE BEGIN PFP */

  /* USER CODE END PFP */

  /* Private user code ---------------------------------------------------------*/
  /* USER CODE BEGIN 0 */

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
    /* USER CODE BEGIN 2 */
    //task01 begin
    // Variables for LSM303AGR initialization
    uint8_t wake_data = 0x67;
    uint8_t mode_data = 0x00;

    // Wake up the accelerometer by writing 0x67 to the CTRL_REG1_A register (0x20)
    HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20, 1, &wake_data, 1, HAL_MAX_DELAY); 

    // Set the accelerometer to Normal mode by writing 0x00 to the CTRL_REG4_A register (0x23)
    HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23, 1, &mode_data, 1, HAL_MAX_DELAY); 
    //task01 end
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // Test UART transmission before any I2C communication starts
    HAL_UART_Transmit(&huart2, (uint8_t*)"Booting\r\n", 9, HAL_MAX_DELAY);
    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      //task02 begin
      // 1. Read I2C Data
      uint8_t x_l = 0, x_h = 0;
      uint8_t y_l = 0, y_h = 0;
      uint8_t z_l = 0, z_h = 0;

      HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x28, 1, &x_l, 1, HAL_MAX_DELAY);
      HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x29, 1, &x_h, 1, HAL_MAX_DELAY);
      HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2A, 1, &y_l, 1, HAL_MAX_DELAY);
      HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2B, 1, &y_h, 1, HAL_MAX_DELAY);
      HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2C, 1, &z_l, 1, HAL_MAX_DELAY);
      HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2D, 1, &z_h, 1, HAL_MAX_DELAY);

      int16_t x_raw = (int16_t)((x_h << 8) | x_l);
      int16_t y_raw = (int16_t)((y_h << 8) | y_l);
      int16_t z_raw = (int16_t)((z_h << 8) | z_l);

      // 2. Calculate Pitch and Roll Angles
    float x_accel = x_raw * 3.9f;
    float y_accel = y_raw * 3.9f;
    float z_accel = z_raw * 3.9f;
    
    // Calculate the raw noisy angle
    float raw_pitch = atan2(-x_accel, sqrt((y_accel * y_accel) + (z_accel * z_accel))) * 57.2958f;
    float raw_roll  = atan2(y_accel, z_accel) * 57.2958f;

    // Apply a Low-Pass Filter (Trust 80% of the old smoothed value, and only 20% of the new noisy value)
    static float pitch_angle = 0.0f; // 'static' retains the value between loop iterations
    static float roll_angle = 0.0f;
    
    pitch_angle = (0.5f * pitch_angle) + (0.5f * raw_pitch);
    roll_angle  = (0.5f * roll_angle)  + (0.5f * raw_roll);

      // 3. Calculate Time Step (dt)
      uint32_t current_time = HAL_GetTick();
      float dt = (current_time - prev_time) / 1000.0f; 
      prev_time = current_time;
      if (dt <= 0.0f) dt = 0.01f; 

      // 4. PID Calculations[cite: 9]
      float error = setpoint - pitch_angle;
      float P_out = Kp * error;

      integral += (error * dt);
      if (integral > 400.0f) integral = 400.0f;
      if (integral < -400.0f) integral = -400.0f;
      float I_out = Ki * integral;

      float derivative = (error - prev_error) / dt;
      float D_out = Kd * derivative;

      float pid_output = P_out + I_out + D_out;
      prev_error = error;

      // 5. Motor Speed and Direction Mapping
      float motor_speed = fabs(pid_output); 
      if (motor_speed > 999.0f) motor_speed = 999.0f; // Constrain to your Timer ARR

      // NOTE: Make sure GPIOD pins 6, 7, 8, 12 and htim3 are configured in CubeMX!
      if (pid_output > 0) 
      {
          // Drive Forward[cite: 113]
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // Right Motor Dir[cite: 18]
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,  GPIO_PIN_SET);   // Left Motor Dir[cite: 18]
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6,  GPIO_PIN_RESET);
      } 
      else 
      {
          // Drive Backward[cite: 113]
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // Right Motor Dir[cite: 18]
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_SET);
          
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,  GPIO_PIN_RESET); // Left Motor Dir[cite: 18]
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6,  GPIO_PIN_SET);
      }

      // Write PWM to motors[cite: 113]
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)motor_speed); // Right PWM (D10)[cite: 18]
      // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)motor_speed); // Left PWM (D9)[cite: 18]

      // 6. UART Transmission for Python Script
      char sensor_buf[60];
      sprintf(sensor_buf, "0,%d,%d,0\r\n", (int)pitch_angle, (int)roll_angle);
      HAL_UART_Transmit(&huart2, (uint8_t*)sensor_buf, strlen(sensor_buf), HAL_MAX_DELAY);

      HAL_Delay(5); // Shortened delay for better PID response time
      //task02 end
      //task01 begin
      // uint8_t x_l = 0, x_h = 0;
      // uint8_t y_l = 0, y_h = 0;
      // uint8_t z_l = 0, z_h = 0;

      // // Read the low and high bytes for the X, Y, and Z axes
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x28, 1, &x_l, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x29, 1, &x_h, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2A, 1, &y_l, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2B, 1, &y_h, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2C, 1, &z_l, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2D, 1, &z_h, 1, HAL_MAX_DELAY);

      // // Combine the bytes into signed 16-bit integers (Must be declared before sprintf)
      // int16_t x_raw = (int16_t)((x_h << 8) | x_l);
      // int16_t y_raw = (int16_t)((y_h << 8) | y_l);
      // int16_t z_raw = (int16_t)((z_h << 8) | z_l);

      // // ---------------------------------------------------------
      // // Modified UART Transmission for Python Script
      // // ---------------------------------------------------------
      // char sensor_buf[60];
      
      // // Output format: Temp, X, Y, Z (casted to int for the Python parser)
      // sprintf(sensor_buf, "0,%d,%d,%d\r\n", (int)x_raw, (int)y_raw, (int)z_raw);
      
      // // Transmit exactly once per loop iteration
      // HAL_UART_Transmit(&huart2, (uint8_t*)sensor_buf, strlen(sensor_buf), HAL_MAX_DELAY);

      // // Delay to prevent terminal flooding (e.g., 100ms)
      // HAL_Delay(100);
      //task01 end
      //task01_old begin
      // // ---------------------------------------------------------
      // // 1. Verify Communication (Your existing test)
      // // ---------------------------------------------------------

      // uint8_t who_am_i = 0;
      // char buffer[30];
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x0F, 1, &who_am_i, 1, HAL_MAX_DELAY);
      // sprintf(buffer, "WHO_AM_I: 0x%02X\r\n", who_am_i);
      // HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
      // // ---------------------------------------------------------
      // // 2. Read Accelerometer Data (The new code)
      // // ---------------------------------------------------------
      // uint8_t x_l = 0, x_h = 0;
      // uint8_t y_l = 0, y_h = 0;
      // uint8_t z_l = 0, z_h = 0;

      // // Read the low and high bytes for the X, Y, and Z axes
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x28, 1, &x_l, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x29, 1, &x_h, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2A, 1, &y_l, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2B, 1, &y_h, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2C, 1, &z_l, 1, HAL_MAX_DELAY);
      // HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x2D, 1, &z_h, 1, HAL_MAX_DELAY);

      // // Combine the bytes into signed 16-bit integers
      // int16_t x_raw = (int16_t)((x_h << 8) | x_l);
      // int16_t y_raw = (int16_t)((y_h << 8) | y_l);
      // int16_t z_raw = (int16_t)((z_h << 8) | z_l);

      // // Scale the raw values to acceleration units (multiplying by 3.9 for LSM303AGR)
      // float x_accel = x_raw * 3.9f;
      // float y_accel = y_raw * 3.9f;
      // float z_accel = z_raw * 3.9f;

      // // Calculate the tilt angle (Pitch)
      // // atan2 returns radians, multiply by 57.2958 to convert to degrees
      // float pitch_angle = atan2(-x_accel, sqrt((y_accel * y_accel) + (z_accel * z_accel))) * 57.2958f;

      // // Calculate Roll:
      // float roll_angle = atan2(y_accel, z_accel) * 57.2958f;
      // // Format and print the scaled values to the terminal
      // char sensor_buf[60];
      // sprintf(sensor_buf, "X: %.2f, Y: %.2f, Z: %.2f\r\n", x_accel, y_accel, z_accel);
      // sprintf(sensor_buf, "Pitch Angle: %.2f degrees\r\n", pitch_angle);
      // HAL_UART_Transmit(&huart2, (uint8_t*)sensor_buf, strlen(sensor_buf), HAL_MAX_DELAY);
      // HAL_UART_Transmit(&huart2, (uint8_t*)sensor_buf, strlen(sensor_buf), HAL_MAX_DELAY);
      // // Delay to prevent terminal flooding (e.g., 100ms)
      // HAL_Delay(100);
      //task01_old end
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
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

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
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                            |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                            |LD6_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                            MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                            |MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                            LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                            LD6_Pin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
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

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
  }

  /* USER CODE BEGIN 4 */

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
