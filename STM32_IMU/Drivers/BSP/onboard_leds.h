/**
  ******************************************************************************
  * @file           : onboard_leds.h
  * @brief          : Header for status LED driver (Board 4: STM32_IMU).
  ******************************************************************************
  */

#ifndef __ONBOARD_LEDS_H
#define __ONBOARD_LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __ONBOARD_LEDS_H */
