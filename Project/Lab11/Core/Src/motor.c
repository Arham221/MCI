/**
 * @file    motor.c
 * @brief   TB6612FNG dual H-bridge driver: direction on two GPIOs, speed on TIM3 PWM.
 *
 *   IN1 IN2 | motor
 *    1   0  | forward
 *    0   1  | reverse
 *    0   0  | stop (coast)
 */
#include "motor.h"
#include "app_config.h"

typedef struct {
  GPIO_TypeDef *in1_port; uint16_t in1_pin;
  GPIO_TypeDef *in2_port; uint16_t in2_pin;
  uint32_t      channel;
  int           invert;
} Motor_Hw_t;

static TIM_HandleTypeDef *s_tim;

static const Motor_Hw_t s_motor[2] = {
  [MOTOR_LEFT]  = { GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_9, TIM_CHANNEL_2, MOTOR_L_INVERT },
  [MOTOR_RIGHT] = { GPIOA, GPIO_PIN_1,  GPIOF, GPIO_PIN_4, TIM_CHANNEL_1, MOTOR_R_INVERT },
};

void Motor_Init(TIM_HandleTypeDef *htim)
{
  s_tim = htim;
  HAL_TIM_PWM_Start(s_tim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(s_tim, TIM_CHANNEL_2);
  Motor_StopAll();
}

void Motor_Set(Motor_Id_t id, float speed)
{
  const Motor_Hw_t *m = &s_motor[id];
  if (m->invert) speed = -speed;

  GPIO_PinState in1 = GPIO_PIN_RESET, in2 = GPIO_PIN_RESET;
  float mag = speed;
  if (speed > 0.5f)       { in1 = GPIO_PIN_SET; }
  else if (speed < -0.5f) { in2 = GPIO_PIN_SET; mag = -speed; }
  else                    { mag = 0.0f; }

  /* Dead-zone compensation: map (0, PWM_MAX] onto (MOTOR_DEADZONE, PWM_MAX] so small
   * controller outputs still overcome the gearbox friction. */
  uint32_t duty = 0;
  if (mag > 0.0f) {
    if (mag > PWM_MAX) mag = PWM_MAX;
    duty = (uint32_t)(MOTOR_DEADZONE + mag * (PWM_MAX - MOTOR_DEADZONE) / (float)PWM_MAX);
  }

  HAL_GPIO_WritePin(m->in1_port, m->in1_pin, in1);
  HAL_GPIO_WritePin(m->in2_port, m->in2_pin, in2);
  __HAL_TIM_SET_COMPARE(s_tim, m->channel, duty);
}

void Motor_SetBoth(float left, float right)
{
  Motor_Set(MOTOR_LEFT, left);
  Motor_Set(MOTOR_RIGHT, right);
}

void Motor_StopAll(void)
{
  Motor_SetBoth(0.0f, 0.0f);
}
