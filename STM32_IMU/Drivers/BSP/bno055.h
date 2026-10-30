/**
  ******************************************************************************
  * @file           : bno055.h
  * @brief          : Driver for Bosch BNO055 9-DOF Intelligent Absolute
  *                   Orientation Sensor (Board 4: STM32_IMU).
  ******************************************************************************
  */

#ifndef __BNO055_H
#define __BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* I2C Address (COM3 pin tied to GND = 0x28, left shifted for HAL = 0x50) */
#define BNO055_I2C_ADDR_LOW         0x28
#define BNO055_I2C_ADDR_HIGH        0x29
#define BNO055_I2C_ADDR             (BNO055_I2C_ADDR_LOW << 1)

/* Page IDs */
#define BNO055_PAGE_ID_ADDR         0x07

/* Register Addresses (PAGE 0) */
#define BNO055_CHIP_ID_ADDR         0x00
#define BNO055_ACC_ID_ADDR          0x01
#define BNO055_MAG_ID_ADDR          0x02
#define BNO055_GYR_ID_ADDR          0x03
#define BNO055_SW_REV_ID_LSB_ADDR   0x04
#define BNO055_SW_REV_ID_MSB_ADDR   0x05
#define BNO055_BL_REV_ID_ADDR       0x06

/* Sensor Data Registers (Page 0) */
#define BNO055_ACC_DATA_X_LSB_ADDR  0x08
#define BNO055_MAG_DATA_X_LSB_ADDR  0x0E
#define BNO055_GYR_DATA_X_LSB_ADDR  0x14
#define BNO055_EUL_HEADING_LSB_ADDR 0x1A
#define BNO055_QUA_DATA_W_LSB_ADDR  0x20
#define BNO055_LIA_DATA_X_LSB_ADDR  0x28
#define BNO055_GRV_DATA_X_LSB_ADDR  0x2E
#define BNO055_TEMP_ADDR            0x34
#define BNO055_CALIB_STAT_ADDR      0x35
#define BNO055_ST_RESULT_ADDR       0x36
#define BNO055_INT_STA_ADDR         0x37
#define BNO055_SYS_CLK_STATUS_ADDR  0x38
#define BNO055_SYS_STATUS_ADDR      0x39
#define BNO055_SYS_ERR_ADDR         0x3A
#define BNO055_UNIT_SEL_ADDR        0x3B
#define BNO055_OPR_MODE_ADDR        0x3D
#define BNO055_PWR_MODE_ADDR        0x3E
#define BNO055_SYS_TRIGGER_ADDR     0x3F
#define BNO055_TEMP_SOURCE_ADDR     0x40
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR   0x42

/* Calibration Offset Registers */
#define BNO055_ACC_OFFSET_X_LSB_ADDR 0x55
#define BNO055_MAG_OFFSET_X_LSB_ADDR 0x5B
#define BNO055_GYR_OFFSET_X_LSB_ADDR 0x61
#define BNO055_ACC_RADIUS_LSB_ADDR   0x67
#define BNO055_MAG_RADIUS_LSB_ADDR   0x69

/* Expected Device IDs */
#define BNO055_CHIP_ID_VAL          0xA0

/* Operation Modes */
typedef enum {
  BNO055_OPR_MODE_CONFIG            = 0x00,
  BNO055_OPR_MODE_ACCONLY           = 0x01,
  BNO055_OPR_MODE_MAGONLY           = 0x02,
  BNO055_OPR_MODE_GYRONLY           = 0x03,
  BNO055_OPR_MODE_ACCMAG            = 0x04,
  BNO055_OPR_MODE_ACCGYRO           = 0x05,
  BNO055_OPR_MODE_MAGGYRO           = 0x06,
  BNO055_OPR_MODE_AMG               = 0x07,
  BNO055_OPR_MODE_IMU               = 0x08,
  BNO055_OPR_MODE_COMPASS           = 0x09,
  BNO055_OPR_MODE_M4G               = 0x0A,
  BNO055_OPR_MODE_NDOF_FMC_OFF      = 0x0B,
  BNO055_OPR_MODE_NDOF              = 0x0C
} BNO055_OprMode_t;

/* Power Modes */
typedef enum {
  BNO055_PWR_MODE_NORMAL            = 0x00,
  BNO055_PWR_MODE_LOWPOWER          = 0x01,
  BNO055_PWR_MODE_SUSPEND           = 0x02
} BNO055_PwrMode_t;

/* Data Structures */
typedef struct {
  int16_t w;
  int16_t x;
  int16_t y;
  int16_t z;
} BNO055_Quaternion_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} BNO055_Vector3_t;

typedef struct {
  uint8_t sys;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
} BNO055_CalibStatus_t;

typedef struct {
  BNO055_Quaternion_t quat;      /* Scaled by 1/16384 (2^14) */
  BNO055_Vector3_t    linear_acc;/* mg (1 mg = 0.00981 m/s^2) or 10^-2 m/s^2 */
  BNO055_Vector3_t    gyro;      /* deg/s or 10^-3 rad/s (16 LSB = 1 deg/s) */
  BNO055_Vector3_t    euler;     /* 16 LSB = 1 degree */
  BNO055_CalibStatus_t calib;    /* 0..3 each */
  int8_t              temperature;
  bool                valid;
} BNO055_Data_t;

/* Calibration Offsets (22 bytes) */
typedef struct {
  uint8_t raw[22];
} BNO055_Offsets_t;

/* Driver API */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c, BNO055_OprMode_t mode);
HAL_StatusTypeDef BNO055_SetMode(I2C_HandleTypeDef *hi2c, BNO055_OprMode_t mode);
HAL_StatusTypeDef BNO055_SetExtCrystal(I2C_HandleTypeDef *hi2c, bool use_ext_crystal);
HAL_StatusTypeDef BNO055_ReadAllData(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data);
HAL_StatusTypeDef BNO055_GetCalibStatus(I2C_HandleTypeDef *hi2c, BNO055_CalibStatus_t *calib);
HAL_StatusTypeDef BNO055_GetOffsets(I2C_HandleTypeDef *hi2c, BNO055_Offsets_t *offsets);
HAL_StatusTypeDef BNO055_SetOffsets(I2C_HandleTypeDef *hi2c, const BNO055_Offsets_t *offsets);
void              BNO055_RecoverBus(void);

#ifdef __cplusplus
}
#endif

#endif /* __BNO055_H */
