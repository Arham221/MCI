/**
 * @file    motor.h
 * @brief   Driver for the two DC motors through the TB6612FNG on the Keyestudio shield.
 *
 * Wiring (STM32 -> shield). Pins are the ones configured in Lab11.ioc; the shield-side
 * D-pin for each IN line follows the manual's pin table (right: D8/D12, left: D7/D6) -
 * check against your jumper wires:
 *   PC6  (TIM3_CH1) -> D10  PWM right motor
 *   PA4  (TIM3_CH2) -> D9   PWM left motor
 *   PA1             -> D8   right IN1
 *   PF4             -> D12  right IN2
 *   PC10            -> D7   left IN1
 *   PC9             -> D6   left IN2
 *   GND             -> GND  (common ground is mandatory)
 * If a wheel turns the wrong way, swap its IN pins or set MOTOR_x_INVERT in app_config.h.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f3xx_hal.h"

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 } Motor_Id_t;

void Motor_Init(TIM_HandleTypeDef *htim);

/** speed in PWM counts, -999..+999. Positive = drive the robot forward. */
void Motor_Set(Motor_Id_t m, float speed);
void Motor_SetBoth(float left, float right);
void Motor_StopAll(void);

#endif /* MOTOR_H */
