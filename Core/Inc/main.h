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
#define SPI_CS_Pin GPIO_PIN_13
#define SPI_CS_GPIO_Port GPIOC
#define SPI_CS2_Pin GPIO_PIN_14
#define SPI_CS2_GPIO_Port GPIOC
#define SPI_CS3_Pin GPIO_PIN_15
#define SPI_CS3_GPIO_Port GPIOC
#define keypadRow1_Pin GPIO_PIN_12
#define keypadRow1_GPIO_Port GPIOB
#define keypadRow2_Pin GPIO_PIN_13
#define keypadRow2_GPIO_Port GPIOB
#define keypadRow3_Pin GPIO_PIN_14
#define keypadRow3_GPIO_Port GPIOB
#define keypadRow4_Pin GPIO_PIN_15
#define keypadRow4_GPIO_Port GPIOB
#define keypadColumn1_Pin GPIO_PIN_9
#define keypadColumn1_GPIO_Port GPIOA
#define keypadColumn2_Pin GPIO_PIN_10
#define keypadColumn2_GPIO_Port GPIOA
#define keypadColumn3_Pin GPIO_PIN_11
#define keypadColumn3_GPIO_Port GPIOA
#define keypadColumn4_Pin GPIO_PIN_12
#define keypadColumn4_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define LTM1_CS_Pin GPIO_PIN_5
#define LTM1_CS_GPIO_Port GPIOB
#define LTM2_CS_Pin GPIO_PIN_6
#define LTM2_CS_GPIO_Port GPIOB
#define LTM3_CS_Pin GPIO_PIN_7
#define LTM3_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define AVAILABLE_CHANNELS 4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
