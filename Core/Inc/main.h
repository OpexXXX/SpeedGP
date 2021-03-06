/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#define  DEBUG_FROM_UART3

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA
#define SD_SPI_CS_Pin GPIO_PIN_6
#define SD_SPI_CS_GPIO_Port GPIOA
#define GPS_EN_Pin GPIO_PIN_0
#define GPS_EN_GPIO_Port GPIOB
#define GPS_PPS_Pin GPIO_PIN_1
#define GPS_PPS_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_15
#define MPU_INT_GPIO_Port GPIOA
#define MPU_INT_EXTI_IRQn EXTI15_10_IRQn
#define BTN_3_Pin GPIO_PIN_3
#define BTN_3_GPIO_Port GPIOB
#define BTN_2_Pin GPIO_PIN_4
#define BTN_2_GPIO_Port GPIOB
#define BTN_1_Pin GPIO_PIN_5
#define BTN_1_GPIO_Port GPIOB
#define LED_LE_Pin GPIO_PIN_6
#define LED_LE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#ifdef DEBUG_FROM_UART3
void UART_Printf(const char* fmt, ...) ;
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
