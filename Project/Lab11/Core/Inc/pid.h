/**
 * @file    pid.h
 * @brief   Position (angle) PID controller with anti-windup and output clamping.
 */
#ifndef PID_H
#define PID_H

typedef struct {
  float kp, ki, kd;     /* gains                                            */
  float out_limit;      /* |output| clamp (PWM counts)                      */
  float i_limit;        /* |integral contribution| clamp (anti-windup)      */
  /* state / last terms (handy for telemetry) */
  float p_term, i_term, d_term, out;
} PID_t;

void  PID_Init(PID_t *c, float kp, float ki, float kd, float out_limit, float i_limit);
void  PID_Reset(PID_t *c);

/**
 * One PID step, called every dt seconds.
 * error = measurement - setpoint (an inverted pendulum is corrected by driving
 * the wheels *towards* the tilt, so a positive tilt must give a positive output).
 * The D term uses the measured rate (gyro) instead of differentiating the error:
 * no derivative kick on set-point changes and much less noise.
 */
float PID_Update(PID_t *c, float setpoint, float measurement, float meas_rate, float dt);

#endif /* PID_H */
