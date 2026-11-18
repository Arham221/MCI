/**
 * @file    imu.h
 * @brief   Driver for the STM32F3 Discovery on-board IMU:
 *          accelerometer LSM303DLHC/LSM303AGR (I2C1) and gyroscope L3GD20/I3G4250D (SPI1).
 */
#ifndef IMU_H
#define IMU_H

#include "stm32f3xx_hal.h"

enum { IMU_X = 0, IMU_Y = 1, IMU_Z = 2 };

typedef struct {
  float acc[3];   /* acceleration in g                      */
  float gyro[3];  /* angular rate in deg/s, bias removed     */
} IMU_Data_t;

/** Configure both sensors (200 Hz output data rate). Returns HAL_OK if both answered. */
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi);

/** Gyro WHO_AM_I register: 0xD4 = L3GD20, 0xD3 = I3G4250D. 0 if no answer. */
uint8_t IMU_GyroWhoAmI(void);

/** Read all six axes (blocking, ~1 ms). Safe to call from the control ISR. */
HAL_StatusTypeDef IMU_Read(IMU_Data_t *d);

/** Average the gyro at rest to remove its zero-rate offset. Robot must be still. */
void IMU_CalibrateGyro(uint16_t samples);

#endif /* IMU_H */
