/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32_IMU (Board 4: AMR IMU Module)
  ******************************************************************************
  */

#include "main.h"
#include "onboard_leds.h"
#include "bno055.h"
#include "imu_app.h"
#include <string.h>
#include <stdio.h>

/* Peripheral Handles */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

static void UART_Log(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 200);
}

static void I2C_Scan(void)
{
  char buf[96];
  UART_Log("\r\n==================================================\r\n");
  UART_Log("[STM32_IMU] Direct UART Telemetry Firmware Booted!\r\n");
  UART_Log("[STM32_IMU] Baud: 115200 | Sample Rate: 100 Hz\r\n");
  UART_Log("[I2C_SCAN] Scanning I2C1 bus (PB6 SCL, PB7 SDA)...\r\n");
  int found = 0;
  for (uint16_t addr = 1; addr < 128; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 2, 5) == HAL_OK)
    {
      snprintf(buf, sizeof(buf), "[I2C_SCAN] Found I2C device at 0x%02X!\r\n", addr);
      UART_Log(buf);
      found++;
    }
  }
  if (found == 0)
  {
    snprintf(buf, sizeof(buf), "[I2C_SCAN] NO I2C devices responded! (HAL ErrorCode: 0x%08lX)\r\n", (unsigned long)HAL_I2C_GetError(&hi2c1));
    UART_Log(buf);
  }
  UART_Log("==================================================\r\n\r\n");
}

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Initialize Status LED immediately for diagnostics */
  LED_Init();
  LED_On();

  /* Configure the system clock to 72 MHz (HSE) or fallback to 64 MHz (HSI) */
  SystemClock_Config();

  /* Pre-initialization I2C bus recovery sequence (clocks SCL 9 times) */
  BNO055_RecoverBus();

  /* Initialize configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* Run I2C Diagnostic Scan & Print Boot Log over UART */
  I2C_Scan();

  /* Visual indication: 2 fast blinks to signal boot completed */
  LED_Off();
  HAL_Delay(100);
  LED_On();
  HAL_Delay(100);
  LED_Off();

  /* Initialize IMU application */
  IMU_App_Init(&hi2c1, &huart1);

  /* Infinite loop */
  while (1)
  {
    IMU_App_Process(&hi2c1, &huart1);
  }
}

/**
  * @brief System Clock Configuration
  *        Tries HSE = 8 MHz + PLL x9 = 72 MHz.
  *        If HSE fails (e.g. unpopulated/cold solder crystal), fallbacks to HSI = 64 MHz.
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Try HSE (External 8 MHz Crystal) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9; /* 8 MHz * 9 = 72 MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Fallback to HSI (Internal 8 MHz RC oscillator) */
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL16; /* 4 MHz * 16 = 64 MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C MSP Initialization
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BNO_I2C_SCL_Pin | BNO_I2C_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
  * @brief I2C MSP De-Initialization
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, BNO_I2C_SCL_Pin | BNO_I2C_SDA_Pin);
  }
}

/**
  * @brief I2C1 Initialization Function (PB6 SCL, PB7 SDA @ 100 kHz)
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000; /* Standard Mode 100 kHz */
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function (PA9 TX, PA10 RX @ 115200 baud)
  */
static void MX_USART1_UART_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* PA9 -> USART1_TX (AF Push-Pull) */
  GPIO_InitStruct.Pin = USART1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USART1_TX_GPIO_Port, &GPIO_InitStruct);

  /* PA10 -> USART1_RX (Input Floating) */
  GPIO_InitStruct.Pin = USART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USART1_RX_GPIO_Port, &GPIO_InitStruct);

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable USART1 interrupt in NVIC for micro-ROS reception */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  /* Port clocks enabled in respective peripheral init functions */
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* Fast toggle on fatal error */
    LED_Toggle();
    for (volatile uint32_t i = 0; i < 500000; i++);
  }
}
