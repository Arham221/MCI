/**
 * @file    telemetry.h
 * @brief   UART2 output (comma-separated, for plotting) and a small command parser for
 *          tuning the controller live, without re-flashing.
 *
 * Output modes ("v <n>"):
 *   0  acc_angle, gyro_rate, filtered_angle, pid_out            (default, Lab 11 plot)
 *   1  ax, ay, az, gx, gy, gz, loop_us                           (raw IMU, axis test)
 *   2  filtered_angle, setpoint, p_term, i_term, d_term, pid_out (tuning)
 *
 * Commands (end with Enter):
 *   e            arm / disarm motors (same as the blue button)
 *   p <x>        set Kp          i <x>  set Ki          d <x>  set Kd
 *   s <x>        set the set-point in degrees
 *   z            set-point = current angle (hold the robot at its balance point first)
 *   m <x>        open-loop motor test, both wheels at duty x (-999..999); "m 0" stops
 *   v <n>        output mode (see above)
 *   ?            print the current parameters
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stm32f3xx_hal.h"

void Telemetry_Init(UART_HandleTypeDef *huart);
void Telemetry_Print(const char *msg);

/** Call often from the main loop: prints telemetry when due and executes commands. */
void Telemetry_Process(void);

/** Must be called from HAL_UART_RxCpltCallback. */
void Telemetry_RxCallback(UART_HandleTypeDef *huart);

#endif /* TELEMETRY_H */
