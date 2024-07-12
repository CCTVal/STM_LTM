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

#define MAX_COMND_COUNT 3 // Numero maximo de veces consecutivas para el mismo caracter
#define THERMOCOUPLES   2
#define CHANNELS        {2, 7}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t cadena[1] = "0";
char last_char = '\0'; // To record last character received
int same_char_count = 0; // Counter for the consecutive times that a specific character has been received
float raw_temperature;
int temperatures[16] = {3500, 3600, 3700, 3800, 3500, 3600, 3700, 3800, 3500, 3600, 3700, 3800, 3500, 3600, 3700, 3800};
uint8_t channels[THERMOCOUPLES] = CHANNELS;
uint8_t i, j;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LTC2986_t therms = {&hspi2, {SPI_CS_GPIO_Port, SPI_CS_Pin}};

void print_status() {
	uint8_t temp_var = LTC2986_read_status(&therms);
	char buffer[32];
	HAL_UART_Transmit(&huart2, (uint8_t *) "status = 0x", strlen("status = 0x"), 100);
	sprintf(buffer, "%02X\n\r", (int) temp_var);
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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(therms.cs_pin.gpio_port, therms.cs_pin.gpio_pin, GPIO_PIN_SET);
  max7219_Init(7);
  max7219_Decode_On();

  HAL_UART_Receive_IT(&huart2, cadena, 1);

  HAL_Delay(4000);
  while(!LTC2986_is_ready(&therms)) {
	print_status();
  }
  max7219_Clean();
  max7219_PrintFtos(DIGIT_8, -3.14, 2);
  max7219_PrintDigit(DIGIT_4, LETTER_H, false);
  max7219_PrintDigit(DIGIT_3, LETTER_E, false);
  max7219_PrintDigit(DIGIT_2, LETTER_L, false);
  max7219_PrintDigit(DIGIT_1, LETTER_P, false);


  // Configure LTM
  LTC2986_configure_thermocouple(&therms, LTC2986_TYPE_T_THERMOCOUPLE, THERMOCOUPLE_CHANNEL, RTD_CHANNEL);
  LTC2986_configure_sense_resistor(&therms, 5, 100);
  LTC2986_configure_rtd(&therms, LTC2986_RTD_PT_100, RTD_CHANNEL, 5);
  LTC2986_global_configure(&therms);
  HAL_Delay(1000);
  max7219_Clean();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// SECTION: Read LTM channels and update temperatures[]
    for(j = 0; j < THERMOCOUPLES; j++) {
    	i = channels[j];
    	raw_temperature = LTC2986_measure_channel(&therms, i);
    	raw_temperature = roundf(raw_temperature * 100) / 100; // We round it up to 2 decimals
    	if(isnan(raw_temperature)) {
    		raw_temperature = 10; // There's been an error
    		// We show the error in the LED display
  			max7219_PrintDigit(j * 4 + 4, LETTER_E, false);
  			max7219_PrintDigit(j * 4 + 3, LETTER_E, false);
  			max7219_PrintDigit(j * 4 + 2, LETTER_E, false);
  			max7219_PrintDigit(j * 4 + 1, LETTER_E, false);
    	} else {
    	    if(raw_temperature > 99.99) raw_temperature = 99.99;
    	    else if(raw_temperature < 10.00) raw_temperature = 10.00;
    	    max7219_PrintFtos(j * 4 + 4, raw_temperature, 2); // We update the LED display
    	}
    	temperatures[j] = (int)(raw_temperature * 100);
    }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

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
    sprintf(temperature_str + (3 + 4*i), "%04d", temperatures[i]);  // falta ver que hacer para que funcione cuando el numero tiene menos de 4 digitos.
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

  /* Prevent unused argument(s) compilation warning */
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
