/**
 * @file    pid.c
 * @brief   Position PID controller.
 */
#include "pid.h"

static float clampf(float x, float lim)
{
  if (x >  lim) return  lim;
  if (x < -lim) return -lim;
  return x;
}

void PID_Init(PID_t *c, float kp, float ki, float kd, float out_limit, float i_limit)
{
  c->kp = kp; c->ki = ki; c->kd = kd;
  c->out_limit = out_limit;
  c->i_limit   = i_limit;
  PID_Reset(c);
}

void PID_Reset(PID_t *c)
{
  c->p_term = c->i_term = c->d_term = c->out = 0.0f;
}

float PID_Update(PID_t *c, float setpoint, float measurement, float meas_rate, float dt)
{
  float error = measurement - setpoint;

  c->p_term = c->kp * error;
  c->d_term = c->kd * meas_rate;            /* d(error)/dt = d(angle)/dt = gyro rate */

  /* Anti-windup 1: conditional integration - stop integrating while the output is
   * saturated and the error would push it further into saturation. */
  int saturated = (c->out >=  c->out_limit && error > 0.0f) ||
                  (c->out <= -c->out_limit && error < 0.0f);
  if (!saturated)
    c->i_term += c->ki * error * dt;

  /* Anti-windup 2: hard clamp on the integral contribution. */
  c->i_term = clampf(c->i_term, c->i_limit);

  c->out = clampf(c->p_term + c->i_term + c->d_term, c->out_limit);
  return c->out;
}
