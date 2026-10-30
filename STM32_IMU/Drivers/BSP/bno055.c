/**
  ******************************************************************************
  * @file           : bno055.c
  * @brief          : Driver implementation for Bosch BNO055 9-DOF Sensor
  *                   (Board 4: STM32_IMU).
  ******************************************************************************
  */

#include "bno055.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

#define BNO055_I2C_TIMEOUT_MS   100

static uint8_t s_bno_i2c_addr = (BNO055_I2C_ADDR_LOW << 1);

static HAL_StatusTypeDef BNO055_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(hi2c, s_bno_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, BNO055_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef BNO055_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
  return HAL_I2C_Mem_Read(hi2c, s_bno_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, val, 1, BNO055_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef BNO055_ReadMulti(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, uint16_t length)
{
  return HAL_I2C_Mem_Read(hi2c, s_bno_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buffer, length, BNO055_I2C_TIMEOUT_MS);
}

/**
  * @brief  Recover I2C bus by clocking SCL 9 times in case a slave is stuck
  */
void BNO055_RecoverBus(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Configure SCL (PB6) and SDA (PB7) as GPIO Open Drain */
  GPIO_InitStruct.Pin = BNO_I2C_SCL_Pin | BNO_I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Set SDA high */
  HAL_GPIO_WritePin(GPIOB, BNO_I2C_SDA_Pin, GPIO_PIN_SET);

  /* Pulse SCL 9 times */
  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, BNO_I2C_SCL_Pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 50; d++);
    HAL_GPIO_WritePin(GPIOB, BNO_I2C_SCL_Pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < 50; d++);
  }

  /* Generate STOP condition */
  HAL_GPIO_WritePin(GPIOB, BNO_I2C_SDA_Pin, GPIO_PIN_RESET);
  for (volatile int d = 0; d < 50; d++);
  HAL_GPIO_WritePin(GPIOB, BNO_I2C_SCL_Pin, GPIO_PIN_SET);
  for (volatile int d = 0; d < 50; d++);
  HAL_GPIO_WritePin(GPIOB, BNO_I2C_SDA_Pin, GPIO_PIN_SET);
  for (volatile int d = 0; d < 50; d++);

  /* Restore PB6 and PB7 to Alternate Function Open Drain for I2C1 */
  GPIO_InitStruct.Pin = BNO_I2C_SCL_Pin | BNO_I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  Initialize BNO055 with external 32.768 kHz crystal and desired fusion mode
  */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c, BNO055_OprMode_t mode)
{
  uint8_t chip_id = 0;
  bool found = false;
  const uint8_t possible_addrs[] = { (BNO055_I2C_ADDR_LOW << 1), (BNO055_I2C_ADDR_HIGH << 1) };

  /* Check Chip ID with retries (BNO055 requires ~650-800ms to boot from POR) */
  for (int retry = 0; retry < 25; retry++)
  {
    for (size_t a = 0; a < sizeof(possible_addrs) / sizeof(possible_addrs[0]); a++)
    {
      s_bno_i2c_addr = possible_addrs[a];
      chip_id = 0;
      HAL_StatusTypeDef status = BNO055_ReadReg(hi2c, BNO055_CHIP_ID_ADDR, &chip_id);
      if (status == HAL_OK && chip_id == BNO055_CHIP_ID_VAL)
      {
        found = true;
        break;
      }
    }
    if (found)
    {
      break;
    }
    HAL_Delay(40);
  }

  if (!found || chip_id != BNO055_CHIP_ID_VAL)
  {
    char err[96];
    snprintf(err, sizeof(err), "[BNO055] Init Failed! chip_id=0x%02X (expected 0xA0)\r\n", chip_id);
    HAL_UART_Transmit(&huart1, (uint8_t *)err, strlen(err), 100);
    return HAL_ERROR;
  }
  else
  {
    char ok[96];
    snprintf(ok, sizeof(ok), "[BNO055] Sensor detected OK at 0x%02X! Chip ID=0x%02X\r\n", s_bno_i2c_addr >> 1, chip_id);
    HAL_UART_Transmit(&huart1, (uint8_t *)ok, strlen(ok), 100);
  }

  /* Switch to CONFIG mode to allow register programming */
  if (BNO055_SetMode(hi2c, BNO055_OPR_MODE_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Select Page 0 */
  if (BNO055_WriteReg(hi2c, BNO055_PAGE_ID_ADDR, 0x00) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set Normal Power Mode */
  if (BNO055_WriteReg(hi2c, BNO055_PWR_MODE_ADDR, BNO055_PWR_MODE_NORMAL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Attempt external 32.768 kHz crystal; if failed, fallback to internal oscillator */
  if (BNO055_SetExtCrystal(hi2c, true) != HAL_OK)
  {
    BNO055_SetExtCrystal(hi2c, false);
  }

  /* Set standard units: m/s^2, deg/s, degrees, Celsius, Windows orientation */
  if (BNO055_WriteReg(hi2c, BNO055_UNIT_SEL_ADDR, 0x00) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Switch to requested operating mode (e.g. NDOF or IMU) */
  if (BNO055_SetMode(hi2c, mode) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Set Operation Mode with appropriate state transition delays
  */
HAL_StatusTypeDef BNO055_SetMode(I2C_HandleTypeDef *hi2c, BNO055_OprMode_t mode)
{
  HAL_StatusTypeDef status = BNO055_WriteReg(hi2c, BNO055_OPR_MODE_ADDR, (uint8_t)mode);
  if (status != HAL_OK)
  {
    return status;
  }

  /* Mode transition delays according to BNO055 datasheet section 3.3 */
  if (mode == BNO055_OPR_MODE_CONFIG)
  {
    HAL_Delay(30);
  }
  else
  {
    HAL_Delay(30);
  }

  return HAL_OK;
}

/**
  * @brief  Enable or disable external 32.768 kHz quartz crystal
  */
HAL_StatusTypeDef BNO055_SetExtCrystal(I2C_HandleTypeDef *hi2c, bool use_ext_crystal)
{
  /* Must be in CONFIG mode */
  uint8_t trigger_val = use_ext_crystal ? 0x80 : 0x00;
  HAL_StatusTypeDef status = BNO055_WriteReg(hi2c, BNO055_SYS_TRIGGER_ADDR, trigger_val);
  HAL_Delay(15);
  return status;
}

/**
  * @brief  Burst read all sensor data (Gyro, Euler, Quaternion, Linear Accel, Temp, Calib)
  */
HAL_StatusTypeDef BNO055_ReadAllData(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data)
{
  if (data == NULL)
  {
    return HAL_ERROR;
  }

  /*
   * Read 34 contiguous bytes from 0x14 (GYR_DATA_X_LSB) to 0x35 (CALIB_STAT):
   * 0x14..0x19 (6B): Gyro X, Y, Z
   * 0x1A..0x1F (6B): Euler Heading, Roll, Pitch
   * 0x20..0x27 (8B): Quaternion W, X, Y, Z
   * 0x28..0x2D (6B): Linear Accel X, Y, Z
   * 0x2E..0x33 (6B): Gravity Vector X, Y, Z (ignored or reserved)
   * 0x34       (1B): Temperature
   * 0x35       (1B): Calibration Status
   */
  uint8_t raw[34];
  HAL_StatusTypeDef status = BNO055_ReadMulti(hi2c, BNO055_GYR_DATA_X_LSB_ADDR, raw, sizeof(raw));

  if (status != HAL_OK)
  {
    data->valid = false;
    return status;
  }

  /* Gyroscope (16 LSB = 1 dps) */
  data->gyro.x = (int16_t)(((int16_t)raw[1] << 8) | raw[0]);
  data->gyro.y = (int16_t)(((int16_t)raw[3] << 8) | raw[2]);
  data->gyro.z = (int16_t)(((int16_t)raw[5] << 8) | raw[4]);

  /* Euler Angles (16 LSB = 1 degree) */
  data->euler.x = (int16_t)(((int16_t)raw[7] << 8) | raw[6]);   /* Heading */
  data->euler.y = (int16_t)(((int16_t)raw[9] << 8) | raw[8]);   /* Roll */
  data->euler.z = (int16_t)(((int16_t)raw[11] << 8) | raw[10]); /* Pitch */

  /* Quaternion (1 LSB = 2^-14) */
  data->quat.w = (int16_t)(((int16_t)raw[13] << 8) | raw[12]);
  data->quat.x = (int16_t)(((int16_t)raw[15] << 8) | raw[14]);
  data->quat.y = (int16_t)(((int16_t)raw[17] << 8) | raw[16]);
  data->quat.z = (int16_t)(((int16_t)raw[19] << 8) | raw[18]);

  /* Linear Acceleration (100 LSB = 1 m/s^2) */
  data->linear_acc.x = (int16_t)(((int16_t)raw[21] << 8) | raw[20]);
  data->linear_acc.y = (int16_t)(((int16_t)raw[23] << 8) | raw[22]);
  data->linear_acc.z = (int16_t)(((int16_t)raw[25] << 8) | raw[24]);

  /* Temperature (1 LSB = 1 C) */
  data->temperature = (int8_t)raw[32];

  /* Calibration Status */
  uint8_t cal = raw[33];
  data->calib.sys   = (cal >> 6) & 0x03;
  data->calib.gyro  = (cal >> 4) & 0x03;
  data->calib.accel = (cal >> 2) & 0x03;
  data->calib.mag   = cal & 0x03;

  data->valid = true;
  return HAL_OK;
}

/**
  * @brief  Read only calibration status register
  */
HAL_StatusTypeDef BNO055_GetCalibStatus(I2C_HandleTypeDef *hi2c, BNO055_CalibStatus_t *calib)
{
  uint8_t val = 0;
  HAL_StatusTypeDef status = BNO055_ReadReg(hi2c, BNO055_CALIB_STAT_ADDR, &val);
  if (status == HAL_OK && calib != NULL)
  {
    calib->sys   = (val >> 6) & 0x03;
    calib->gyro  = (val >> 4) & 0x03;
    calib->accel = (val >> 2) & 0x03;
    calib->mag   = val & 0x03;
  }
  return status;
}

/**
  * @brief  Read 22-byte calibration offsets
  */
HAL_StatusTypeDef BNO055_GetOffsets(I2C_HandleTypeDef *hi2c, BNO055_Offsets_t *offsets)
{
  if (offsets == NULL)
  {
    return HAL_ERROR;
  }
  /* Must be read in CONFIG mode */
  BNO055_SetMode(hi2c, BNO055_OPR_MODE_CONFIG);
  HAL_StatusTypeDef status = BNO055_ReadMulti(hi2c, BNO055_ACC_OFFSET_X_LSB_ADDR, offsets->raw, 22);
  return status;
}

/**
  * @brief  Write 22-byte calibration offsets
  */
HAL_StatusTypeDef BNO055_SetOffsets(I2C_HandleTypeDef *hi2c, const BNO055_Offsets_t *offsets)
{
  if (offsets == NULL)
  {
    return HAL_ERROR;
  }
  /* Must be written in CONFIG mode */
  BNO055_SetMode(hi2c, BNO055_OPR_MODE_CONFIG);
  return HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_ACC_OFFSET_X_LSB_ADDR,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)offsets->raw, 22, BNO055_I2C_TIMEOUT_MS);
}
