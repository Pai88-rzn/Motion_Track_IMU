/**
  ******************************************************************************
  * @file           : microros_imu.h
  * @brief          : micro-ROS (Jazzy) client node publishing /imu @ 100 Hz
  *                   (Board 4: STM32_IMU).
  ******************************************************************************
  */

#ifndef __MICROROS_IMU_H
#define __MICROROS_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bno055.h"

typedef enum {
  MICROROS_STATE_WAITING_AGENT = 0,
  MICROROS_STATE_AGENT_AVAILABLE,
  MICROROS_STATE_AGENT_CONNECTED,
  MICROROS_STATE_AGENT_DISCONNECTED
} MicroRosState_t;

void            MicroRosIMU_Init(UART_HandleTypeDef *huart);
void            MicroRosIMU_Process(const BNO055_Data_t *sensor_data);
MicroRosState_t MicroRosIMU_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __MICROROS_IMU_H */
