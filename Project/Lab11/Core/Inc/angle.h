/**
 * @file    angle.h
 * @brief   Tilt-angle estimation: accelerometer angle + complementary filter.
 */
#ifndef ANGLE_H
#define ANGLE_H

#include "imu.h"

typedef struct {
  float acc_angle;   /* tilt from the accelerometer only (deg) - noisy, no drift  */
  float gyro_rate;   /* rate about the wheel axle (deg/s)                         */
  float angle;       /* complementary-filtered tilt (deg)                         */
} Angle_t;

/** Start the filter from the accelerometer angle so it doesn't need to converge. */
void  Angle_Init(Angle_t *a, const IMU_Data_t *d);

/** One filter step; must be called every DT seconds. */
void  Angle_Update(Angle_t *a, const IMU_Data_t *d);

/** Accelerometer-only tilt angle in degrees (uses the axes from app_config.h). */
float Angle_FromAccel(const IMU_Data_t *d);

/** Gyro rate about the wheel axle in deg/s (uses the axis from app_config.h). */
float Angle_RateFromGyro(const IMU_Data_t *d);

#endif /* ANGLE_H */
