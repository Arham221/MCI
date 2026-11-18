/**
 * @file    control.c
 * @brief   200 Hz balance loop: IMU -> complementary filter -> PID -> motors.
 *
 * Everything that needs a constant dt lives here and is executed from the TIM6
 * interrupt, so UART printing in the main loop can never disturb the sample time.
 */
#include "control.h"
#include "motor.h"
#include "app_config.h"
#include "main.h"
#include <math.h>

static IMU_Data_t   s_imu;
static Angle_t      s_ang;
static PID_t        s_pid;
static volatile float        s_setpoint   = SETPOINT_DEFAULT;
static volatile Ctrl_State_t s_state      = CTRL_DISARMED;
static volatile float        s_test_duty  = 0.0f;
static volatile uint8_t      s_tlm_due    = 0;
static volatile uint32_t     s_loop_us    = 0;
static volatile uint32_t     s_imu_errors = 0;

void Control_Init(void)
{
  /* DWT cycle counter: used only to measure how long one control step takes. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  PID_Init(&s_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, PID_OUT_LIMIT, PID_I_LIMIT);
  if (IMU_Read(&s_imu) != HAL_OK) s_imu_errors++;
  Angle_Init(&s_ang, &s_imu);
}

static void set_led(uint16_t pin, int on)
{
  HAL_GPIO_WritePin(GPIOE, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Control_Step(void)
{
  static uint8_t tlm_div = 0;
  uint32_t t0 = DWT->CYCCNT;

  /* 1) sensors + 2) complementary filter (same DT as the PID) */
  if (IMU_Read(&s_imu) == HAL_OK)
    Angle_Update(&s_ang, &s_imu);
  else
    s_imu_errors++;

  float err = s_ang.angle - s_setpoint;

  /* 3) PID on the filtered angle - always computed so it can be plotted, even disarmed */
  PID_Update(&s_pid, s_setpoint, s_ang.angle, s_ang.gyro_rate, DT);

  /* 4) state machine + motors */
  switch (s_state) {
    case CTRL_DISARMED:
      Motor_StopAll();
      PID_Reset(&s_pid);
      break;

    case CTRL_WAITING:                       /* armed: start once (almost) upright */
      Motor_StopAll();
      PID_Reset(&s_pid);
      if (fabsf(err) < REARM_ANGLE_DEG) s_state = CTRL_BALANCING;
      break;

    case CTRL_BALANCING:
      if (fabsf(err) > FALL_ANGLE_DEG) {     /* fallen over: cut motors, wait for pick-up */
        Motor_StopAll();
        PID_Reset(&s_pid);
        s_state = CTRL_WAITING;
      } else {
        Motor_SetBoth(s_pid.out, s_pid.out);
      }
      break;

    case CTRL_MOTOR_TEST:
      Motor_SetBoth(s_test_duty, s_test_duty);
      break;
  }

  /* status LEDs: LD3 = armed, LD10 = balancing, LD7 = tilted forward, LD6 = tilted back */
  set_led(LD3_Pin,  s_state != CTRL_DISARMED);
  set_led(LD10_Pin, s_state == CTRL_BALANCING);
  set_led(LD7_Pin,  err >  1.0f);
  set_led(LD6_Pin,  err < -1.0f);

  /* 5) telemetry is printed by the main loop at CONTROL_HZ / TELEMETRY_DIV */
  if (++tlm_div >= TELEMETRY_DIV) { tlm_div = 0; s_tlm_due = 1; }

  s_loop_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
}

uint8_t Control_TelemetryDue(void)
{
  if (!s_tlm_due) return 0;
  s_tlm_due = 0;
  return 1;
}

void Control_GetSnapshot(Ctrl_Snapshot_t *s)
{
  __disable_irq();                           /* copy atomically w.r.t. the ISR */
  s->imu        = s_imu;
  s->ang        = s_ang;
  s->setpoint   = s_setpoint;
  s->p          = s_pid.p_term;
  s->i          = s_pid.i_term;
  s->d          = s_pid.d_term;
  s->out        = s_pid.out;
  s->state      = s_state;
  s->loop_us    = s_loop_us;
  s->imu_errors = s_imu_errors;
  __enable_irq();
}

void Control_SetArmed(uint8_t on)
{
  s_state = on ? CTRL_WAITING : CTRL_DISARMED;
}

void Control_ToggleArm(void)
{
  Control_SetArmed(s_state == CTRL_DISARMED);
}

void Control_SetGains(float kp, float ki, float kd)
{
  __disable_irq();
  s_pid.kp = kp; s_pid.ki = ki; s_pid.kd = kd;
  s_pid.i_term = 0.0f;
  __enable_irq();
}

PID_t Control_GetPid(void)
{
  __disable_irq();
  PID_t c = s_pid;
  __enable_irq();
  return c;
}

void  Control_SetSetpoint(float deg) { s_setpoint = deg; }
float Control_GetSetpoint(void)      { return s_setpoint; }
void  Control_ZeroSetpoint(void)     { s_setpoint = s_ang.angle; }

void Control_MotorTest(float duty)
{
  if (duty == 0.0f) {
    s_state = CTRL_DISARMED;
  } else {
    s_test_duty = duty;
    s_state = CTRL_MOTOR_TEST;
  }
}
