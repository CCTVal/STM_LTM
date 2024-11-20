/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LTC2986.h"
#include "max7219.h"
#include "keypad.h"
#include "i2c_lcd.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define THERMOCOUPLE_CHANNEL (uint8_t) 0x02
#define RTD_CHANNEL (uint8_t) 0x07
#define SENSE_CHANNEL (uint8_t) 0x05

#define MAX_COMND_COUNT 3 // Numero maximo de veces consecutivas para el mismo caracter

#define THERMOMETRY_CHIPS 2
#define THERMOCOUPLES   2
#define THERMOCOUPLES_CHANNELS        {0x02, 0x12}

typedef enum {
	NORMAL_STATE = 0x0,
	CALIBRATE_ALL_PROBES_STATE = 0x1,
	SHOULD_PROBE_STATE = 0x2,
	CONFIRM_PROBES_STATE = 0x3,
	ONE_OR_TWO_STATE = 0x4,
	PLACE_ONLY_POINT_STATE = 0x5,
	INPUT_ONLY_TEMPERATURE_STATE = 0x6,
	PLACE_FIRST_POINT_STATE = 0x7,
	INPUT_FIRST_TEMPERATURE_STATE = 0x8,
	PLACE_SECOND_POINT_STATE = 0x9,
	INPUT_SECOND_TEMPERATURE_STATE = 0xA,
	CALIBRATION_READY = 0xB,
	CRITICAL_ERROR = 0xF
} state_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t cadena[1] = "0";
char last_char = '\0'; // To record last character received
int same_char_count = 0; // Counter for the consecutive times that a specific character has been received
float raw_temperature;
int temperatures[16] = {3500, 3600, 3700, 3800, 3500, 3600, 3700, 3800, 3500, 3600, 3700, 3800, 3500, 3600, 3700, 3800};
uint8_t thermocouple_channels[THERMOCOUPLES] = THERMOCOUPLES_CHANNELS;
uint8_t i, j;
state_t operation_state = NORMAL_STATE;

// Calibration process variables
uint8_t probe_to_ponder = 0;
uint8_t calibration_state = 0;
float calibration_temperature1 = 0;
float calibration_temperature2 = 0;
char LCD_buffer[34];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LTC2986_t therms[THERMOMETRY_CHIPS] = {
		{&hspi2, {LTM1_CS_GPIO_Port, LTM1_CS_Pin}},
		{&hspi2, {LTM2_CS_GPIO_Port, LTM2_CS_Pin}}
};

void print_status(int chip) {
	uint8_t temp_var = LTC2986_read_status(therms + chip);
	char buffer[48];
	HAL_UART_Transmit(&huart2, (uint8_t *) "status = 0x", strlen("status = 0x"), 100);
	sprintf(buffer, "%02X\n\r chip = ", (unsigned int) temp_var);
	HAL_UART_Transmit(&huart2, (uint8_t *) buffer, strlen(buffer), 100);
	sprintf(buffer, "%01X\n\r", chip);
	HAL_UART_Transmit(&huart2, (uint8_t *) buffer, strlen(buffer), 100);
	HAL_Delay(200);
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  max7219_Init(3);
  max7219_Decode_On();

  HAL_Delay(4000);
  max7219_Clean();
  max7219_PrintFtos(DIGIT_1, -3.14, 2);
  max7219_PrintDigit(DIGIT_5, LETTER_H, false);
  max7219_PrintDigit(DIGIT_6, LETTER_E, false);
  max7219_PrintDigit(DIGIT_7, LETTER_L, false);
  max7219_PrintDigit(DIGIT_8, LETTER_P, false);
  max7219_PrintFtos(9, -4.25, 2);
  max7219_PrintDigit(13, LETTER_E, false);
  max7219_PrintDigit(14, LETTER_H, false);
  max7219_PrintDigit(15, LETTER_E, false);
  max7219_PrintDigit(16, LETTER_H, false);
  HAL_Delay(1000);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //max7219_Clean();
	  HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI_CS_Pin|SPI_CS2_Pin|SPI_CS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|keypadColumn1_Pin|keypadColumn2_Pin|keypadColumn3_Pin
                          |keypadColumn4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LTM1_CS_Pin|LTM2_CS_Pin|LTM3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_CS_Pin SPI_CS2_Pin SPI_CS3_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|SPI_CS2_Pin|SPI_CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin keypadColumn1_Pin keypadColumn2_Pin keypadColumn3_Pin
                           keypadColumn4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|keypadColumn1_Pin|keypadColumn2_Pin|keypadColumn3_Pin
                          |keypadColumn4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : keypadRow1_Pin keypadRow2_Pin keypadRow3_Pin keypadRow4_Pin */
  GPIO_InitStruct.Pin = keypadRow1_Pin|keypadRow2_Pin|keypadRow3_Pin|keypadRow4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LTM1_CS_Pin LTM2_CS_Pin LTM3_CS_Pin */
  GPIO_InitStruct.Pin = LTM1_CS_Pin|LTM2_CS_Pin|LTM3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t char_counter(char c) {
  // Si el carácter recibido es diferente al último carácter almacenado
  if (c != last_char) {
    // Reiniciar el contador y actualizar el último carácter almacenado
    same_char_count = 1;
    last_char = c;
  } else {
    // Si es el mismo carácter, incrementar el contador
    same_char_count++;
  }

  // Si se ha recibido el mismo carácter el número máximo de veces consecutivas
  if (same_char_count >= MAX_COMND_COUNT) {
    // Reiniciar el contador y el último carácter almacenado
    same_char_count = 0;
    last_char = '\0';
    return 1;
  } else {
    return 0;
  }
}

void N_name_and_status_handler(){
  char respuesta_N[6]="111000"; // This is for LT-100. Comment for S-100
  //char respuesta_N[6]="222000"; // This is for S-100. Comment for LT-100
  char *s;
  for ( s=respuesta_N; *s != '\0'; s++ ) {
    HAL_UART_Transmit(&huart2,(uint8_t *)s , 1, 100);
  }
}

void done(){
  char respuesta_N[3]="DDD";
  char *s;
  for ( s=respuesta_N; *s != '\0'; s++ ) {
    HAL_UART_Transmit(&huart2,(uint8_t *)s , 1, 100);
  }
}

void T_temperature_handler(){
  char temperature_str[70] = "EEE";
  int i=0;
  for(i=0;i<16;i++){
    temperature_str[i*4 + 6]= '\0';
    if(temperatures[i]) {
      sprintf(temperature_str + (3 + 4*i), "%04d", temperatures[i]);
    } else {  // error
      sprintf(temperature_str + (3 + 4*i), "0000");
    }
  }

  unsigned checksum = 0;
  for (int i = 3; i < 3 + 4 * 16; i++) {
    checksum += temperature_str[i];
  }
  int onChar = 3+64;
  temperature_str[onChar++] = checksum % 256;
  temperature_str[onChar++] = checksum / 256;

  char *s;
  for ( s=temperature_str; *s != '\0'; s++ ) {
    HAL_UART_Transmit(&huart2,(uint8_t *)s , 1, 100);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // Prevent unused argument(s) compilation warning
  if(huart->Instance == USART2)
  {
      switch(cadena[0]){
        case 'N':
          if(char_counter(cadena[0])){
            //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
            N_name_and_status_handler();
          }
          break;
        case 'T':
          if(char_counter(cadena[0])){
             T_temperature_handler();
          }
          break;
        case 'R':
          if(char_counter(cadena[0])){
            T_temperature_handler();
          }
          break;

        default:
          //HAL_UART_Transmit(&huart2, (uint8_t *)"Comando no reconocido\n", strlen("Comando no reconocido\n"), 100); // acá se debe poner una función tipo "No_recognized_command_handler"
          break;
      }
    HAL_UART_Receive_IT(&huart2, cadena, 1);
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint16_t key = getKeyAsInt(GPIO_Pin);
  if(key == KEYPAD_ERROR_KEY) {
    return;
  }
  if(key == 0xB) {
	operation_state = NORMAL_STATE;
	lcd_print("Cancel");
    return;
  }
  switch(operation_state) {
    case NORMAL_STATE:
      if(key == 0xC) {
	    operation_state = CALIBRATE_ALL_PROBES_STATE;
	    lcd_print("Calibrate all\nprobes? (yes/no)");
      }
	  break;
    case CALIBRATE_ALL_PROBES_STATE:
      if(key == 1 || key == 0xC) { // Yes
    	operation_state = CONFIRM_PROBES_STATE;
    	lcd_print("Confirm probes");
      } else if(key == 2) { // No
    	operation_state = SHOULD_PROBE_STATE;


    	probe_to_ponder = 0;
    	sprintf(LCD_buffer, "Calibrate probe\n%d? (yes/no)", ++probe_to_ponder );
    	lcd_print(LCD_buffer);
      }
      break;
    case SHOULD_PROBE_STATE:
      if(key == 1 || key == 0xC) { // Yes
    	// Probe should be considered
      } else if(key == 2) { // No
    	// Probe should not be considered
      }
      sprintf(LCD_buffer, "Calibrate probe\n%d? (yes/no)", ++probe_to_ponder );
      if(probe_to_ponder > 15) {
        operation_state = CONFIRM_PROBES_STATE;
        lcd_print("Confirm probes");
        break;
      }
      lcd_print(LCD_buffer);
      break;
    case CONFIRM_PROBES_STATE:
      if(key == 0xC) {
        operation_state = ONE_OR_TWO_STATE;
        lcd_print("How many points?\n(1/2)");
      }
      break;
    case ONE_OR_TWO_STATE:
      if(key == 1) {
        operation_state = PLACE_ONLY_POINT_STATE;
        lcd_print("Place probes to\nref. temp."); // print "place probes to reference temperature point"
      } else if(key == 2) {
        operation_state = PLACE_FIRST_POINT_STATE;
        lcd_print("Place probes to\nref. temp. 1"); // print "place probes to reference temperature point"
      }
      break;
    case PLACE_ONLY_POINT_STATE:
      if(key == 0xC) {
        operation_state = INPUT_ONLY_TEMPERATURE_STATE;
        lcd_print("Input\ntemperature");
      }
      break;
    case INPUT_ONLY_TEMPERATURE_STATE:
      if(key == 0xC) {
    	calibration_state = 1; // one point calibration
    	calibration_temperature1 = key * 10;
        operation_state = CALIBRATION_READY;
        lcd_print("Calibration\nready!");
      } else {
    	calibration_temperature1 = key * 10; // TODO
      }
      break;
    case PLACE_FIRST_POINT_STATE:
      operation_state = INPUT_FIRST_TEMPERATURE_STATE;
      break;
    case INPUT_FIRST_TEMPERATURE_STATE:
      operation_state = PLACE_SECOND_POINT_STATE;
      break;
    case PLACE_SECOND_POINT_STATE:
      operation_state = INPUT_SECOND_TEMPERATURE_STATE;
      break;
    case INPUT_SECOND_TEMPERATURE_STATE:
      operation_state = CALIBRATION_READY;
      break;

    default:
	    operation_state = NORMAL_STATE;
	    lcd_clear();
	    break;
    }
  return;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
