/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC_PWR_Pin GPIO_PIN_0
#define PC_PWR_GPIO_Port GPIOF
#define SCREEN_PWR_Pin GPIO_PIN_1
#define SCREEN_PWR_GPIO_Port GPIOF
#define BTN2_Pin GPIO_PIN_0
#define BTN2_GPIO_Port GPIOA
#define VIM_IN_Pin GPIO_PIN_1
#define VIM_IN_GPIO_Port GPIOA
#define HUB_PWR_Pin GPIO_PIN_3
#define HUB_PWR_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOA
#define WIRE_Pin GPIO_PIN_5
#define WIRE_GPIO_Port GPIOA
#define VIM_BTN_Pin GPIO_PIN_6
#define VIM_BTN_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_7
#define FAN_GPIO_Port GPIOA
#define SELF_PWR_Pin GPIO_PIN_1
#define SELF_PWR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
