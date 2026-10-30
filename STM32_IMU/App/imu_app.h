/**
  ******************************************************************************
  * @file           : imu_app.h
  * @brief          : Header for main IMU supervisory application
  *                   (Board 4: STM32_IMU).
  ******************************************************************************
  */

#ifndef __IMU_APP_H
#define __IMU_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bno055.h"
#include "serial_protocol.h"
#include "onboard_leds.h"

typedef enum {
  IMU_STATE_UNINITIALIZED = 0,
  IMU_STATE_INIT,
  IMU_STATE_STREAMING,
  IMU_STATE_ERROR
} ImuAppState_t;

typedef struct {
  ImuAppState_t       state;
  BNO055_Data_t       sensor_data;
  ImuTelemetryPacket_t packet;
  uint32_t            sample_period_ms;  /* Default: 10 ms (100 Hz) */
  uint32_t            last_sample_tick;
  uint32_t            last_led_tick;
  uint32_t            packets_sent;
  uint32_t            error_count;
} ImuApp_t;

void          IMU_App_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void          IMU_App_Process(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
ImuAppState_t IMU_App_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_APP_H */
