/**
 * @file    control.h
 * @brief   Fixed-rate balance control loop (runs in the TIM6 interrupt at CONTROL_HZ).
 *
 *   TIM6 ISR (every DT):  read IMU -> complementary filter -> PID -> motors -> telemetry flag
 *   main loop          :  button, UART commands, UART printing (slow, non-critical work)
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "imu.h"
#include "angle.h"
#include "pid.h"

typedef enum {
  CTRL_DISARMED = 0,   /* motors off (blue button / "e" command toggles)        */
  CTRL_WAITING,        /* armed, waiting for the robot to be held near upright  */
  CTRL_BALANCING,      /* closed loop running                                   */
  CTRL_MOTOR_TEST      /* open-loop motor test ("m <duty>" command)             */
} Ctrl_State_t;

/** Consistent copy of the loop's variables for printing. */
typedef struct {
  IMU_Data_t   imu;
  Angle_t      ang;
  float        setpoint;
  float        p, i, d, out;
  Ctrl_State_t state;
  uint32_t     loop_us;      /* execution time of one control step   */
  uint32_t     imu_errors;   /* failed IMU reads since boot          */
} Ctrl_Snapshot_t;

/** Initialise motors/filter/PID. IMU must be initialised and calibrated before. */
void Control_Init(void);

/** The control step - call from HAL_TIM_PeriodElapsedCallback for TIM6 only. */
void Control_Step(void);

/** Returns 1 (and clears it) every TELEMETRY_DIV control steps. */
uint8_t Control_TelemetryDue(void);
void    Control_GetSnapshot(Ctrl_Snapshot_t *s);

/* ---- run-time parameters (from the button / UART commands) ---- */
void  Control_ToggleArm(void);
void  Control_SetArmed(uint8_t on);
void  Control_SetGains(float kp, float ki, float kd);
PID_t Control_GetPid(void);
void  Control_SetSetpoint(float deg);
float Control_GetSetpoint(void);
void  Control_ZeroSetpoint(void);          /* setpoint = current filtered angle */
void  Control_MotorTest(float duty);       /* 0 ends the test                   */

#endif /* CONTROL_H */
