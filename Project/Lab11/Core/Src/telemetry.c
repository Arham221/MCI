/**
 * @file    telemetry.c
 * @brief   UART telemetry + command interface (see telemetry.h for the command list).
 */
#include "telemetry.h"
#include "control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_MAX 32

static UART_HandleTypeDef *s_uart;
static uint8_t           s_rx_byte;
static char              s_line[LINE_MAX];
static volatile uint8_t  s_line_len   = 0;
static volatile uint8_t  s_line_ready = 0;
static uint8_t           s_mode       = 0;

void Telemetry_Init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);
}

void Telemetry_Print(const char *msg)
{
  HAL_UART_Transmit(s_uart, (const uint8_t *)msg, strlen(msg), 50);
}

/* RX interrupt: collect one line, hand it to the main loop. */
void Telemetry_RxCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_uart) return;
  char c = (char)s_rx_byte;

  if (!s_line_ready) {
    if (c == '\r' || c == '\n') {
      if (s_line_len) { s_line[s_line_len] = '\0'; s_line_ready = 1; }
    } else if (s_line_len < LINE_MAX - 1) {
      s_line[s_line_len++] = c;
    }
  }
  HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);
}

/* Keep receiving after overrun/noise errors (e.g. while a long print is going out). */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == s_uart) HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);
}

static void print_params(void)
{
  char buf[160];
  PID_t c = Control_GetPid();
  snprintf(buf, sizeof buf, "# Kp=%.3f Ki=%.3f Kd=%.3f setpoint=%.2f mode=%u\r\n",
           c.kp, c.ki, c.kd, Control_GetSetpoint(), s_mode);
  Telemetry_Print(buf);
}

static void execute(const char *cmd)
{
  char  op  = cmd[0];
  float val = (strlen(cmd) > 1) ? strtof(cmd + 1, NULL) : 0.0f;
  PID_t c   = Control_GetPid();

  switch (op) {
    case 'e': Control_ToggleArm(); break;
    case 'p': Control_SetGains(val, c.ki, c.kd); break;
    case 'i': Control_SetGains(c.kp, val, c.kd); break;
    case 'd': Control_SetGains(c.kp, c.ki, val); break;
    case 's': Control_SetSetpoint(val); break;
    case 'z': Control_ZeroSetpoint(); break;
    case 'm': Control_MotorTest(val); break;
    case 'v': s_mode = (uint8_t)val; break;
    case '?': break;
    default:  Telemetry_Print("# unknown command\r\n"); return;
  }
  print_params();                 /* echo the new state (lines starting with '#' are ignored by the plotter) */
}

static void print_telemetry(void)
{
  Ctrl_Snapshot_t s;
  char buf[128];
  int n;

  Control_GetSnapshot(&s);
  switch (s_mode) {
    case 1:
      n = snprintf(buf, sizeof buf, "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%lu\r\n",
                   s.imu.acc[0], s.imu.acc[1], s.imu.acc[2],
                   s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                   (unsigned long)s.loop_us);
      break;
    case 2:
      n = snprintf(buf, sizeof buf, "%.2f,%.2f,%.1f,%.1f,%.1f,%.1f\r\n",
                   s.ang.angle, s.setpoint, s.p, s.i, s.d, s.out);
      break;
    default:
      n = snprintf(buf, sizeof buf, "%.2f,%.2f,%.2f,%.1f\r\n",
                   s.ang.acc_angle, s.ang.gyro_rate, s.ang.angle, s.out);
      break;
  }
  if (n > 0) HAL_UART_Transmit(s_uart, (uint8_t *)buf, (uint16_t)n, 20);
}

void Telemetry_Process(void)
{
  if (s_line_ready) {
    char cmd[LINE_MAX];
    strcpy(cmd, s_line);
    s_line_len = 0;
    s_line_ready = 0;
    execute(cmd);
  }
  if (Control_TelemetryDue()) print_telemetry();
}
