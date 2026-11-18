/**
 * @file    app_config.h
 * @brief   Every tunable parameter of the Lab 11 / balancing firmware in one place.
 *
 * Hardware: STM32F3 Discovery (STM32F303VCT6) + Keyestudio Ks0193 balance shield (TB6612FNG).
 *  - Accelerometer: on-board LSM303DLHC / LSM303AGR, I2C1 (PB6/PB7), address 0x32
 *  - Gyroscope    : on-board L3GD20 / I3G4250D, SPI1 (PA5/PA6/PA7), chip select PE3
 *  - Control loop : TIM6 update interrupt, 48 MHz / 48 / 5000 = 200 Hz
 *  - Motor PWM    : TIM3, 48 MHz / 48 / 1000 = 1 kHz, ARR = 999
 *  - Telemetry    : USART2 (PA2 = TX, PA3 = RX), 115200 8N1
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ======================= Timing ======================= */
/* Must match TIM6: f = 48 MHz / (PSC+1) / (ARR+1) = 48e6 / 48 / 5000 = 200 Hz.
 * The SAME DT is used by the complementary filter and by the PID controller. */
#define CONTROL_HZ          200
#define DT                  (1.0f / (float)CONTROL_HZ)   /* 0.005 s */
#define TELEMETRY_DIV       10      /* 200 Hz / 10 = 20 lines per second over UART */

/* ======================= Angle estimation ======================= */
#define COMP_ALPHA          0.98f   /* angle = a*(angle + gyro*dt) + (1-a)*acc_angle
                                     * time constant tau = a*dt/(1-a) = 0.245 s          */
#define GYRO_CAL_SAMPLES    400     /* 400 x 5 ms = 2 s of averaging at start-up (keep still!) */

/*
 * Axis selection -- depends on how the Discovery board is mounted on the robot.
 * Run the firmware, type "v 1" in the terminal (raw mode) and look at the numbers:
 *   ACC_UP_AXIS    : the accelerometer axis that reads about +1 g / -1 g while the robot stands upright
 *   ACC_FWD_AXIS   : the accelerometer axis that changes the most when the robot tips forward/backward
 *   GYRO_AXLE_AXIS : the gyro axis parallel to the wheel axle (the one that reacts when you tip it)
 * Then use ANGLE_SIGN so that tipping FORWARD gives a POSITIVE angle, and
 * GYRO_SIGN so that the gyro rate is POSITIVE while tipping forward.
 *
 *   Board lying flat (chip side up), wheel axle along board X  -> UP=Z, FWD=Y, GYRO=X
 *   Board lying flat (chip side up), wheel axle along board Y  -> UP=Z, FWD=X, GYRO=Y
 *   Board standing vertical, USB edge up, axle along board X   -> UP=Y, FWD=Z, GYRO=X
 *
 * Note: accelerometer and gyro must describe the SAME rotation: rotation about X shows up
 * in the Y/Z accelerometer axes, rotation about Y shows up in the X/Z axes, and so on.
 */
/* Measured on the robot (v 1 test, 2026-11-18): board flat on top, axle along board Y.
 * Upright az = +0.96 g; tipping forward -> ax negative, gy negative. */
#define ACC_UP_AXIS         IMU_Z
#define ACC_FWD_AXIS        IMU_X
#define GYRO_AXLE_AXIS      IMU_Y
#define ANGLE_SIGN          (1.0f)
#define GYRO_SIGN           (-1.0f)

/* ======================= PID controller ======================= */
/* Output units: PWM counts (-999..+999).  Angle in degrees, rate in deg/s.
 * error = angle - setpoint ; positive output drives the wheels "forward" (= under the tilt). */
#define PID_KP_DEFAULT      60.0f   /* counts per degree                                */
#define PID_KI_DEFAULT      0.0f    /* counts per (degree * second) -- start at 0 (PD)  */
#define PID_KD_DEFAULT      1.5f    /* counts per (degree / second)                     */
#define SETPOINT_DEFAULT    0.0f    /* degrees; type "z" while holding the robot at its balance point */
#define PID_OUT_LIMIT       999.0f  /* = TIM3 ARR (100 % duty)                          */
#define PID_I_LIMIT         300.0f  /* anti-windup clamp on the integral contribution   */

/* ======================= Safety / arming ======================= */
#define FALL_ANGLE_DEG      40.0f   /* |angle - setpoint| beyond this -> motors off      */
#define REARM_ANGLE_DEG     3.0f    /* after a fall, restart only once lifted within this */

/* ======================= Motors (TB6612FNG on the shield) ======================= */
#define PWM_MAX             999     /* TIM3 ARR                                          */
#define MOTOR_DEADZONE      0       /* duty below which the wheels don't turn; measure with "m <duty>" */
#define MOTOR_L_INVERT      0       /* set to 1 if this wheel turns the wrong way for "m 300" */
#define MOTOR_R_INVERT      0

#endif /* APP_CONFIG_H */
