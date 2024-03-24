/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define I_SENSE_A_Pin GPIO_PIN_0
#define I_SENSE_A_GPIO_Port GPIOC
#define I_SENSE_B_Pin GPIO_PIN_1
#define I_SENSE_B_GPIO_Port GPIOC
#define I_SENSE_C_Pin GPIO_PIN_2
#define I_SENSE_C_GPIO_Port GPIOC
#define V_IN_VOLT_Pin GPIO_PIN_3
#define V_IN_VOLT_GPIO_Port GPIOC
#define V_SENSE_A_Pin GPIO_PIN_0
#define V_SENSE_A_GPIO_Port GPIOA
#define V_SENCE_B_Pin GPIO_PIN_1
#define V_SENCE_B_GPIO_Port GPIOA
#define V_SENCE_C_Pin GPIO_PIN_2
#define V_SENCE_C_GPIO_Port GPIOA
#define NTC_TEMP_A_Pin GPIO_PIN_3
#define NTC_TEMP_A_GPIO_Port GPIOA
#define TEST2_Pin GPIO_PIN_4
#define TEST2_GPIO_Port GPIOA
#define TEST1_Pin GPIO_PIN_5
#define TEST1_GPIO_Port GPIOA
#define MCU_AIN_INPIT_Pin GPIO_PIN_5
#define MCU_AIN_INPIT_GPIO_Port GPIOC
#define NTC_TEMP_B_Pin GPIO_PIN_0
#define NTC_TEMP_B_GPIO_Port GPIOB
#define NTC_TEMP_C_Pin GPIO_PIN_1
#define NTC_TEMP_C_GPIO_Port GPIOB
#define PHASE_VOLT_FILT_Pin GPIO_PIN_9
#define PHASE_VOLT_FILT_GPIO_Port GPIOC
#define PHASE_CUR_FILT_Pin GPIO_PIN_2
#define PHASE_CUR_FILT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
