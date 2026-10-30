/**
  ******************************************************************************
  * @file           : custom_transport.h
  * @brief          : Custom UART transport for micro-ROS on STM32F103 (USART1)
  ******************************************************************************
  */

#ifndef __CUSTOM_TRANSPORT_H
#define __CUSTOM_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

void custom_transport_init(UART_HandleTypeDef *huart);
void custom_transport_rx_push(uint8_t byte);

bool custom_transport_open(struct uxrCustomTransport * transport);
bool custom_transport_close(struct uxrCustomTransport * transport);
size_t custom_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_TRANSPORT_H */
