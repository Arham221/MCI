/**
 * @file    angle.c
 * @brief   Tilt-angle estimation with a complementary filter.
 *
 *   angle = 0.98 * (angle + gyro_rate * dt) + 0.02 * acc_angle
 *
 * The gyro term follows fast motion but drifts (integrated bias); the accelerometer term
 * is drift-free but noisy and disturbed by the robot's own acceleration. The 0.98/0.02
 * weights make the gyro dominate below ~0.25 s and the accelerometer dominate beyond it.
 */
#include "angle.h"
#include "app_config.h"
#include <math.h>

#define RAD2DEG 57.2957795f

float Angle_FromAccel(const IMU_Data_t *d)
{
  /* Tipping forward by theta: the up-axis reads cos(theta) g and the forward axis
   * reads -sin(theta) g, so theta = atan2(-fwd, up). ANGLE_SIGN fixes mounting. */
  return ANGLE_SIGN * atan2f(-d->acc[ACC_FWD_AXIS], d->acc[ACC_UP_AXIS]) * RAD2DEG;
}

float Angle_RateFromGyro(const IMU_Data_t *d)
{
  return GYRO_SIGN * d->gyro[GYRO_AXLE_AXIS];
}

void Angle_Init(Angle_t *a, const IMU_Data_t *d)
{
  a->acc_angle = Angle_FromAccel(d);
  a->gyro_rate = 0.0f;
  a->angle     = a->acc_angle;
}

void Angle_Update(Angle_t *a, const IMU_Data_t *d)
{
  a->acc_angle = Angle_FromAccel(d);
  a->gyro_rate = Angle_RateFromGyro(d);
  a->angle     = COMP_ALPHA * (a->angle + a->gyro_rate * DT)
               + (1.0f - COMP_ALPHA) * a->acc_angle;
}
