/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   STM32_IMU (Board 4: AMR IMU Module)
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
/* Onboard Status LED2 (Active LOW) */
#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOC

/* USART1 to FT231XS USB-to-UART */
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA

/* I2C1 to Bosch BNO055 */
#define BNO_I2C_SCL_Pin GPIO_PIN_6
#define BNO_I2C_SCL_GPIO_Port GPIOB
#define BNO_I2C_SDA_Pin GPIO_PIN_7
#define BNO_I2C_SDA_GPIO_Port GPIOB

#ifndef HSE_VALUE
#define HSE_VALUE ((uint32_t)8000000U) /*!< Value of the External oscillator in Hz */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
