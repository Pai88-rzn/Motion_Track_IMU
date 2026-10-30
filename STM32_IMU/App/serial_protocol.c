/**
  ******************************************************************************
  * @file           : serial_protocol.c
  * @brief          : Implementation of serial binary telemetry frame protocol
  *                   (Board 4: STM32_IMU).
  ******************************************************************************
  */

#include "serial_protocol.h"
#include <string.h>

#define UART_TX_TIMEOUT_MS  10

void SerialProtocol_Init(UART_HandleTypeDef *huart)
{
  (void)huart;
}

/**
  * @brief  Standard CRC-16-CCITT calculation (poly 0x1021, init 0xFFFF)
  */
uint16_t SerialProtocol_ComputeCRC16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }

  return crc;
}

/**
  * @brief  Build telemetry packet from BNO055 sensor data and MCU timestamp
  */
void SerialProtocol_BuildPacket(const BNO055_Data_t *sensor_data, uint32_t timestamp, ImuTelemetryPacket_t *pkt)
{
  if (sensor_data == NULL || pkt == NULL)
  {
    return;
  }

  pkt->sync[0]      = IMU_PKT_SYNC0;
  pkt->sync[1]      = IMU_PKT_SYNC1;
  pkt->timestamp_ms = timestamp;

  pkt->qw           = sensor_data->quat.w;
  pkt->qx           = sensor_data->quat.x;
  pkt->qy           = sensor_data->quat.y;
  pkt->qz           = sensor_data->quat.z;

  pkt->ax           = sensor_data->linear_acc.x;
  pkt->ay           = sensor_data->linear_acc.y;
  pkt->az           = sensor_data->linear_acc.z;

  pkt->gx           = sensor_data->gyro.x;
  pkt->gy           = sensor_data->gyro.y;
  pkt->gz           = sensor_data->gyro.z;

  pkt->yaw          = sensor_data->euler.x;
  pkt->roll         = sensor_data->euler.y;
  pkt->pitch        = sensor_data->euler.z;

  pkt->calib_stat   = ((sensor_data->calib.sys   & 0x03) << 6) |
                      ((sensor_data->calib.gyro  & 0x03) << 4) |
                      ((sensor_data->calib.accel & 0x03) << 2) |
                      (sensor_data->calib.mag    & 0x03);

  pkt->temp_c       = sensor_data->temperature;

  /* Compute CRC16 over all bytes excluding the 2-byte CRC16 field itself */
  uint16_t payload_len = (uint16_t)(sizeof(ImuTelemetryPacket_t) - sizeof(uint16_t));
  pkt->crc16 = SerialProtocol_ComputeCRC16((const uint8_t *)pkt, payload_len);
}

/**
  * @brief  Transmit serialized telemetry frame over UART
  */
HAL_StatusTypeDef SerialProtocol_SendPacket(UART_HandleTypeDef *huart, const ImuTelemetryPacket_t *pkt)
{
  if (huart == NULL || pkt == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(huart, (uint8_t *)pkt, sizeof(ImuTelemetryPacket_t), UART_TX_TIMEOUT_MS);
}
