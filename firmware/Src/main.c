/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "stdlib.h"
#include <stdbool.h>
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const uint8_t SUSPEND_TIME=10;
const uint8_t SHUTDOWN_TIME=30;
const uint16_t SHUTDOWN_KEYPRESS_TIME=1000; //milliseconds
const uint16_t SHUTDOWN_CHECK_INTERVAL=20; //seconds
const uint8_t VIM_START_DELAY=10; //seconds
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */


struct screen_str {
	bool enabled;
};

struct acc_str {
	bool enabled;
	bool low;
	uint16_t voltage;
	uint32_t time_to_suspend;
	uint32_t time_to_shutdown;
};

struct vim_str {
	bool enabled;
	bool sleep;
	bool voltage;
	uint8_t shutdown_count;
};

struct itps_str {
	bool enabled;
	uint32_t time_to_action;
};

struct hub_str {
	bool enabled;
};
struct screen_str SCREEN;
struct vim_str VIM;
struct itps_str ITPS;
struct hub_str HUB;
struct acc_str ACC;

uint32_t now, next;
float t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t readADC() {
	uint16_t adc = 0;
	HAL_ADC_Start(&hadc);
	if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK) {
		Error_Handler();
	} else {
		adc = HAL_ADC_GetValue(&hadc);
	}
	HAL_ADC_Stop(&hadc);
	return adc;
}

bool checkVIM() {
	VIM.voltage = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	if (VIM.voltage == true)
		return 1;
	else
		return 0;
}

void setPWM(uint16_t duty) {
	if (duty >= 124)  // Max PWM dutycycle
		duty = 124;
	if (duty <= 0)
		duty = 0; //Min PWM dutycycle
	TIM3->CCR1 = duty;
}

void EnableScreen(bool force) {
	if (SCREEN.enabled == false) {
		HAL_GPIO_WritePin(SCREEN_PWR_GPIO_Port, SCREEN_PWR_Pin, GPIO_PIN_SET);
		SCREEN.enabled = true;
	}
	if (force) {
		HAL_GPIO_WritePin(SCREEN_PWR_GPIO_Port, SCREEN_PWR_Pin, GPIO_PIN_SET);
		SCREEN.enabled = true;
	}
}

void EnableVIM(bool force) {
	if (VIM.enabled == false) {
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PC_PWR_GPIO_Port, PC_PWR_Pin, GPIO_PIN_SET);
		VIM.enabled = true;
	}
	if (force) {
		HAL_GPIO_WritePin(PC_PWR_GPIO_Port, PC_PWR_Pin, GPIO_PIN_SET);
		VIM.enabled = true;
	}
}

void EnableHUB(bool force) {
	if (HUB.enabled == false) {
		HAL_GPIO_WritePin(HUB_PWR_GPIO_Port, HUB_PWR_Pin, GPIO_PIN_SET);
		HUB.enabled = true;
	}
	if (force) {
		HAL_GPIO_WritePin(HUB_PWR_GPIO_Port, HUB_PWR_Pin, GPIO_PIN_SET);
		HUB.enabled = true;
	}
}

void DisableScreen(bool force) {
	if (SCREEN.enabled == true) {
		HAL_GPIO_WritePin(SCREEN_PWR_GPIO_Port, SCREEN_PWR_Pin, GPIO_PIN_RESET);
		SCREEN.enabled = false;
	}
	if (force) {
		HAL_GPIO_WritePin(SCREEN_PWR_GPIO_Port, SCREEN_PWR_Pin, GPIO_PIN_RESET);
		SCREEN.enabled = false;
	}
}

void DisableVIM(bool force) {
	if (VIM.enabled == true) {
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PC_PWR_GPIO_Port, PC_PWR_Pin, GPIO_PIN_RESET);
		VIM.enabled = false;
		VIM.sleep = false;
	}
	if (force) {
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PC_PWR_GPIO_Port, PC_PWR_Pin, GPIO_PIN_RESET);
		VIM.enabled = false;
		VIM.sleep = false;
	}
}

void DisableHUB(bool force) {
	if (HUB.enabled == true) {
		HAL_GPIO_WritePin(HUB_PWR_GPIO_Port, HUB_PWR_Pin, GPIO_PIN_RESET);
		HUB.enabled = false;
	}
	if (force) {
		HAL_GPIO_WritePin(HUB_PWR_GPIO_Port, HUB_PWR_Pin, GPIO_PIN_RESET);
		HUB.enabled = false;
	}
}

void DisableITPS() {
	HAL_GPIO_WritePin(SELF_PWR_GPIO_Port, SELF_PWR_Pin, GPIO_PIN_RESET);
	ITPS.enabled = false;
}

void SuspendVIM() {
	if (VIM.sleep == false) {
		HAL_GPIO_WritePin(VIM_BTN_GPIO_Port, VIM_BTN_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(VIM_BTN_GPIO_Port, VIM_BTN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
		VIM.sleep = true;
	}
}

void ShutdownVIM() {
	HAL_GPIO_WritePin(VIM_BTN_GPIO_Port, VIM_BTN_Pin, GPIO_PIN_SET);
	HAL_Delay(SHUTDOWN_KEYPRESS_TIME);
	HAL_GPIO_WritePin(VIM_BTN_GPIO_Port, VIM_BTN_Pin, GPIO_PIN_RESET);
}

void ResumeVIM() {
	if (VIM.sleep == true) {
		HAL_GPIO_WritePin(VIM_BTN_GPIO_Port, VIM_BTN_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(VIM_BTN_GPIO_Port, VIM_BTN_Pin, GPIO_PIN_RESET);
		VIM.sleep = false;
	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  	  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; //тактируем таймер
  	  TIM16->CR1 |= TIM_CR1_CEN; //включаем
	HAL_ADC_Stop(&hadc);
	HAL_ADCEx_Calibration_Start(&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	VIM.shutdown_count=0;
	HAL_GPIO_WritePin(SELF_PWR_GPIO_Port, SELF_PWR_Pin, GPIO_PIN_SET);
	ITPS.enabled = true;
	HUB.enabled = false;
	SCREEN.enabled = false;
	VIM.enabled = false;
	VIM.sleep = false;
	HAL_Delay(2000);
	while (readADC() < 2814) {
	}
	EnableScreen(false);
	HAL_Delay(VIM_START_DELAY*1000);
	if (readADC() >2814)
		EnableVIM(false);
	else
		DisableITPS();		//disable WHOLE Supply voltage
	HAL_Delay(2000);
	EnableHUB(false);
	while (1) {
		ACC.voltage = readADC();
		//every 100ms
		if (HAL_GetTick() >= next) {
			ACC.voltage = readADC();
			checkVIM();
			if (ACC.voltage < 3169 && ACC.time_to_suspend == 0 && ACC.time_to_shutdown == 0) //ACC Voltage below 11v
			{
				if (VIM.enabled==false)
				{
					DisableVIM(false);	//disable VIM Supply voltage
					HAL_Delay(10);
					DisableHUB(false);	//disable HUB Supply voltage
					HAL_Delay(10);
					DisableScreen(false);
					HAL_Delay(100);	//disable SCREEN Supply voltage
					DisableITPS();		//disable WHOLE Supply voltage
				}
				else
				{
					ACC.enabled = false;
					ACC.low = true;
					ACC.time_to_suspend = HAL_GetTick() + SUSPEND_TIME * 1000; //10 seconds to suspend
					ACC.time_to_shutdown = ACC.time_to_suspend + SHUTDOWN_TIME * 1000; //5 hours to shutdown
				}
			}
			if (ACC.voltage > 3169) //if ACC voltage > 12.2V! cancel suspend an resume carpc
					{
				ACC.enabled = true;
				ACC.low = false;
				ACC.time_to_suspend = 0; //
				ACC.time_to_shutdown = 0; //
			}
			next = HAL_GetTick() + 100;
		}
		if (HAL_GetTick() >= ACC.time_to_suspend && ACC.time_to_suspend != 0) {
			SuspendVIM(false);
			HAL_Delay(5000); // 5 seconds to suspend
			DisableHUB(false);
			HAL_Delay(2000);
			DisableScreen(false);
		}
		if (HAL_GetTick() >= ACC.time_to_shutdown && ACC.time_to_shutdown != 0) {
			EnableScreen(false);
			HAL_Delay(2 * 1000);
			ShutdownVIM();  //VIM starts to resume from sleep
			HAL_Delay(5 * 1000); //5 seconds to resuming
			while (checkVIM()) { // checking voltage on GPIO27 VIM's pin.
				ShutdownVIM();	// if still HIGH try shutdown again
				HAL_Delay(SHUTDOWN_CHECK_INTERVAL * 1000); //~20 seconds to shutdown OS on vim
				VIM.shutdown_count++;	//counter of shutdown tries
				if (VIM.shutdown_count>5) // maximum 4 retries
				{
					DisableVIM(false); 		//and hard off
					break;
				}
			}
			HAL_Delay(2 * 1000);
			DisableVIM(false);	//disable VIM Supply voltage
			HAL_Delay(2 * 1000);
			DisableHUB(false);	//disable HUB Supply voltage
			HAL_Delay(1000);
			DisableScreen(false);
			HAL_Delay(1000);	//disable SCREEN Supply voltage
			DisableITPS();		//disable WHOLE Supply voltage
		}
		if (ACC.time_to_suspend == 0 && VIM.sleep == true) {
			EnableScreen(false);
			HAL_Delay(2 * 1000);
			ResumeVIM(false);
			HAL_Delay(4 * 1000); //4 seconds to resume
			EnableHUB(false);
		}
		if (ACC.time_to_shutdown == 0 && VIM.enabled == false) {
			EnableScreen(false);
			HAL_Delay(2 * 1000);
			EnableVIM(false);
			HAL_Delay(2 * 1000);
			EnableHUB(false);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, PC_PWR_Pin|SCREEN_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HUB_PWR_Pin|VIM_BTN_Pin|FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SELF_PWR_GPIO_Port, SELF_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC_PWR_Pin SCREEN_PWR_Pin */
  GPIO_InitStruct.Pin = PC_PWR_Pin|SCREEN_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN2_Pin VIM_IN_Pin BTN1_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin|VIM_IN_Pin|BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HUB_PWR_Pin VIM_BTN_Pin FAN_Pin */
  GPIO_InitStruct.Pin = HUB_PWR_Pin|VIM_BTN_Pin|FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SELF_PWR_Pin */
  GPIO_InitStruct.Pin = SELF_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SELF_PWR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
