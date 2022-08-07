/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define IMU_INT_Pin GPIO_PIN_3
#define IMU_INT_GPIO_Port GPIOE
#define BUZZ_ON_Pin GPIO_PIN_7
#define BUZZ_ON_GPIO_Port GPIOA
#define STATUS_LED1_R_Pin GPIO_PIN_8
#define STATUS_LED1_R_GPIO_Port GPIOE
#define STATUS_LED1_G_Pin GPIO_PIN_9
#define STATUS_LED1_G_GPIO_Port GPIOE
#define STATUS_LED1_B_Pin GPIO_PIN_10
#define STATUS_LED1_B_GPIO_Port GPIOE
#define STATUS_LED2_R_Pin GPIO_PIN_11
#define STATUS_LED2_R_GPIO_Port GPIOE
#define STATUS_LED2_G_Pin GPIO_PIN_12
#define STATUS_LED2_G_GPIO_Port GPIOE
#define STATUS_LED2_B_Pin GPIO_PIN_13
#define STATUS_LED2_B_GPIO_Port GPIOE
#define PS_INT_Pin GPIO_PIN_15
#define PS_INT_GPIO_Port GPIOE
#define USB_VBUS_DEBUG_Pin GPIO_PIN_10
#define USB_VBUS_DEBUG_GPIO_Port GPIOD
#define SD_CARD_DETECT_Pin GPIO_PIN_8
#define SD_CARD_DETECT_GPIO_Port GPIOA
#define MAG_INT_Pin GPIO_PIN_0
#define MAG_INT_GPIO_Port GPIOE
#define LED_EXT_Pin GPIO_PIN_1
#define LED_EXT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
