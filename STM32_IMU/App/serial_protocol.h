/**
  ******************************************************************************
  * @file           : serial_protocol.h
  * @brief          : Serial binary telemetry frame protocol definition
  *                   (Board 4: STM32_IMU).
  ******************************************************************************
  */

#ifndef __SERIAL_PROTOCOL_H
#define __SERIAL_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bno055.h"

#define IMU_PKT_SYNC0   0x55
#define IMU_PKT_SYNC1   0xAA

#pragma pack(push, 1)
typedef struct {
  uint8_t  sync[2];       /* 0x55, 0xAA */
  uint32_t timestamp_ms;  /* Milliseconds since MCU boot */
  int16_t  qw;            /* Quaternion W (1 LSB = 2^-14) */
  int16_t  qx;            /* Quaternion X */
  int16_t  qy;            /* Quaternion Y */
  int16_t  qz;            /* Quaternion Z */
  int16_t  ax;            /* Linear Acceleration X (100 LSB = 1 m/s^2) */
  int16_t  ay;            /* Linear Acceleration Y */
  int16_t  az;            /* Linear Acceleration Z */
  int16_t  gx;            /* Angular Velocity X (16 LSB = 1 deg/s) */
  int16_t  gy;            /* Angular Velocity Y */
  int16_t  gz;            /* Angular Velocity Z */
  int16_t  yaw;           /* Euler Heading (16 LSB = 1 deg) */
  int16_t  roll;          /* Euler Roll */
  int16_t  pitch;         /* Euler Pitch */
  uint8_t  calib_stat;    /* Sys[7:6], Gyro[5:4], Acc[3:2], Mag[1:0] */
  int8_t   temp_c;        /* Temperature in Celsius */
  uint16_t crc16;         /* CRC16-CCITT across all preceding bytes */
} ImuTelemetryPacket_t;
#pragma pack(pop)

void     SerialProtocol_Init(UART_HandleTypeDef *huart);
uint16_t SerialProtocol_ComputeCRC16(const uint8_t *data, uint16_t len);
void     SerialProtocol_BuildPacket(const BNO055_Data_t *sensor_data, uint32_t timestamp, ImuTelemetryPacket_t *pkt);
HAL_StatusTypeDef SerialProtocol_SendPacket(UART_HandleTypeDef *huart, const ImuTelemetryPacket_t *pkt);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_PROTOCOL_H */
