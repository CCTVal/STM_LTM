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
#define keypadRow1_EXTI_IRQn EXTI15_10_IRQn
#define keypadRow2_Pin GPIO_PIN_13
#define keypadRow2_GPIO_Port GPIOB
#define keypadRow2_EXTI_IRQn EXTI15_10_IRQn
#define keypadRow3_Pin GPIO_PIN_14
#define keypadRow3_GPIO_Port GPIOB
#define keypadRow3_EXTI_IRQn EXTI15_10_IRQn
#define keypadRow4_Pin GPIO_PIN_15
#define keypadRow4_GPIO_Port GPIOB
#define keypadRow4_EXTI_IRQn EXTI15_10_IRQn
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
// LTM setting
#define AVAILABLE_CHANNELS 16

// Calibration states
typedef enum {
	NORMAL_STATE = 0x00,
    CALIBRATE_ALL_PROBES_STATE = 0x01,
    CONFIRM_DETECTED_STATE = 0x02,
    SHOULD_PROBE_STATE = 0x03,
    CONFIRM_PROBES_STATE = 0x04,
    ONE_OR_TWO_STATE = 0x05,
    PLACE_ONLY_POINT_STATE = 0x06,
    INPUT_ONLY_TEMPERATURE_STATE = 0x07,
    PLACE_FIRST_POINT_STATE = 0x08,
    INPUT_FIRST_TEMPERATURE_STATE = 0x09,
    PLACE_SECOND_POINT_STATE = 0x0A,
    INPUT_SECOND_TEMPERATURE_STATE = 0x0B,
    CALIBRATION_COMPLETE_STATE = 0x0C
} calibration_state_t;

typedef enum {
	FACTORY_CALIBRATED = 0x01,
	PREVIOUSLY_CALIBRATED = 0x02,
	JUST_CALIBRATED = 0x04,
    ONE_POINT_CALIBRATED = 0x10,
    TWO_POINT_CALIBRATED = 0x20,
} calibrated_t;

void update_temperatures();
void calibrate_all_probes_handler();
void confirm_detected_handler();
void should_probe_handler();
void confirm_probes_handler();
void one_or_two_handler();
void place_only_point_handler();
void input_only_temperature_handler();
void place_first_point_handler();
void input_first_temperature_handler();
void place_second_point_handler();
void input_second_temperature_handler();
void calibration_complete_handler();

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
