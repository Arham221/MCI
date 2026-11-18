/**
 * @file    imu.c
 * @brief   On-board IMU driver (accelerometer over I2C, gyroscope over SPI).
 */
#include "imu.h"

/* ---------- Accelerometer (LSM303DLHC / LSM303AGR) ---------- */
#define ACC_ADDR            0x32    /* 0x19 << 1                                     */
#define ACC_CTRL_REG1_A     0x20
#define ACC_CTRL_REG4_A     0x23
#define ACC_OUT_X_L_A       0x28
#define ACC_AUTO_INC        0x80    /* MSB of sub-address = auto increment           */
#define ACC_REG1_200HZ_XYZ  0x67    /* ODR = 200 Hz, normal mode, X/Y/Z enabled       */
#define ACC_REG4_BDU_2G_HR  0x88    /* block data update, +-2 g, high resolution     */
#define ACC_LSB_PER_G       16384.0f /* +-2 g, data left-justified in 16 bits         */

/* ---------- Gyroscope (L3GD20 / I3G4250D) ---------- */
#define GYRO_WHO_AM_I       0x0F
#define GYRO_CTRL_REG1      0x20
#define GYRO_CTRL_REG4      0x23
#define GYRO_OUT_X_L        0x28
#define GYRO_READ           0x80
#define GYRO_AUTO_INC       0x40
#define GYRO_REG1_200HZ_XYZ 0x4F    /* ODR ~190-200 Hz, power on, X/Y/Z enabled       */
#define GYRO_REG4_BDU_245   0x80    /* block data update, +-245 dps                   */
#define GYRO_DPS_PER_LSB    0.00875f /* 8.75 mdps/LSB at +-245 dps                     */

#define GYRO_CS_PORT        GPIOE
#define GYRO_CS_PIN         GPIO_PIN_3
#define CS_LOW()            HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()           HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

#define BUS_TIMEOUT_MS      3

static I2C_HandleTypeDef *s_i2c;
static SPI_HandleTypeDef *s_spi;
static float s_gyro_bias[3];

static HAL_StatusTypeDef gyro_write(uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = { reg, val };
  CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(s_spi, tx, 2, BUS_TIMEOUT_MS);
  CS_HIGH();
  return st;
}

static HAL_StatusTypeDef gyro_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
  uint8_t cmd = GYRO_READ | (n > 1 ? GYRO_AUTO_INC : 0) | reg;
  CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(s_spi, &cmd, 1, BUS_TIMEOUT_MS);
  if (st == HAL_OK) st = HAL_SPI_Receive(s_spi, buf, n, BUS_TIMEOUT_MS);
  CS_HIGH();
  return st;
}

static HAL_StatusTypeDef acc_write(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(s_i2c, ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
}

/* Raw read without bias removal (used by calibration as well). */
static HAL_StatusTypeDef read_raw(IMU_Data_t *d)
{
  uint8_t a[6], g[6];

  if (HAL_I2C_Mem_Read(s_i2c, ACC_ADDR, ACC_OUT_X_L_A | ACC_AUTO_INC,
                       I2C_MEMADD_SIZE_8BIT, a, 6, BUS_TIMEOUT_MS) != HAL_OK)
    return HAL_ERROR;
  if (gyro_read(GYRO_OUT_X_L, g, 6) != HAL_OK)
    return HAL_ERROR;

  for (int i = 0; i < 3; i++) {
    int16_t ra = (int16_t)((a[2 * i + 1] << 8) | a[2 * i]);
    int16_t rg = (int16_t)((g[2 * i + 1] << 8) | g[2 * i]);
    d->acc[i]  = ra / ACC_LSB_PER_G;
    d->gyro[i] = rg * GYRO_DPS_PER_LSB;
  }
  return HAL_OK;
}

HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef st = HAL_OK;
  s_i2c = hi2c;
  s_spi = hspi;
  CS_HIGH();

  if (acc_write(ACC_CTRL_REG1_A, ACC_REG1_200HZ_XYZ) != HAL_OK) st = HAL_ERROR;
  if (acc_write(ACC_CTRL_REG4_A, ACC_REG4_BDU_2G_HR) != HAL_OK) st = HAL_ERROR;

  if (gyro_write(GYRO_CTRL_REG1, GYRO_REG1_200HZ_XYZ) != HAL_OK) st = HAL_ERROR;
  if (gyro_write(GYRO_CTRL_REG4, GYRO_REG4_BDU_245) != HAL_OK) st = HAL_ERROR;
  if (IMU_GyroWhoAmI() == 0x00 || IMU_GyroWhoAmI() == 0xFF) st = HAL_ERROR;

  HAL_Delay(50);                  /* let both sensors produce their first samples */
  return st;
}

uint8_t IMU_GyroWhoAmI(void)
{
  uint8_t id = 0;
  if (gyro_read(GYRO_WHO_AM_I, &id, 1) != HAL_OK) return 0;
  return id;
}

HAL_StatusTypeDef IMU_Read(IMU_Data_t *d)
{
  if (read_raw(d) != HAL_OK) return HAL_ERROR;
  for (int i = 0; i < 3; i++) d->gyro[i] -= s_gyro_bias[i];
  return HAL_OK;
}

void IMU_CalibrateGyro(uint16_t samples)
{
  float sum[3] = { 0 };
  uint16_t n = 0;
  IMU_Data_t d;

  for (uint16_t k = 0; k < samples; k++) {
    if (read_raw(&d) == HAL_OK) {
      for (int i = 0; i < 3; i++) sum[i] += d.gyro[i];
      n++;
    }
    HAL_Delay(5);
  }
  for (int i = 0; i < 3; i++) s_gyro_bias[i] = n ? sum[i] / n : 0.0f;
}
