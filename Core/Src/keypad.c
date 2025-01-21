/*
 * keypad.c
 *
 *  Created on: Jun 21, 2024
 *      Author: vsaona
 */

#include "keypad.h"

uint32_t previousMillis = 0;

void keypad_Init(){
  HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_SET);
}

uint16_t getKeyAsInt(uint16_t GPIO_Pin) {
  uint16_t row, column;
  row = (KEYPAD_ERROR_KEY & 0xFF00) >> 2;
  column = KEYPAD_ERROR_KEY & 0x00FF;
  uint16_t button = KEYPAD_ERROR_KEY;
  uint32_t currentMillis;
  GPIO_TypeDef* port;
  switch(GPIO_Pin) {
	  case keypadRow1_Pin:
		row = 0;
		port = keypadRow1_GPIO_Port;
		break;
	  case keypadRow2_Pin:
		row = 1;
		port = keypadRow2_GPIO_Port;
	  	break;
	  case keypadRow3_Pin:
		row = 2;
		port = keypadRow3_GPIO_Port;
	  	break;
	  case keypadRow4_Pin:
		row = 3;
		port = keypadRow4_GPIO_Port;
	  	break;
  }
  currentMillis = HAL_GetTick();

  if (currentMillis - previousMillis > 100) {
	if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	  return(KEYPAD_ERROR_KEY);
	}
	HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_RESET);
	if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	  column = 1;
	}
	HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_RESET);
	if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	  column = 2;
	}
	HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_RESET);
	if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	  column = 3;
	}
	HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_RESET);
	if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	  column = 4;
	}
	HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_SET);
	previousMillis = currentMillis;
	button = column + row * 4;
	/*
	if(button == 10) { // asterisk
		button = 11;
	} else if (button == 11) { // zero
		button = 0;
	} else if (button == 12) { // hash
		button = 12;
	}*/
  }
  return(button);
}

uint16_t getPressedKey() {
  uint16_t row, column;
  row = (KEYPAD_ERROR_KEY & 0xFF00) >> 2;
  column = KEYPAD_ERROR_KEY & 0x00FF;
  uint16_t button = KEYPAD_ERROR_KEY;
  uint16_t pin;
  GPIO_TypeDef* port;
  if(HAL_GPIO_ReadPin(keypadRow1_GPIO_Port, keypadRow1_Pin)) {
	  row = 0;
	  port = keypadRow1_GPIO_Port;
	  pin = keypadRow1_Pin;
  } else if(HAL_GPIO_ReadPin(keypadRow2_GPIO_Port, keypadRow2_Pin)) {
	  row = 1;
	  port = keypadRow2_GPIO_Port;
	  pin = keypadRow2_Pin;
  } else if(HAL_GPIO_ReadPin(keypadRow3_GPIO_Port, keypadRow3_Pin)) {
	  row = 2;
	  port = keypadRow3_GPIO_Port;
	  pin = keypadRow3_Pin;
  } else if(HAL_GPIO_ReadPin(keypadRow4_GPIO_Port, keypadRow4_Pin)) {
	  row = 3;
	  port = keypadRow4_GPIO_Port;
	  pin = keypadRow4_Pin;
  } else {
	  return(KEYPAD_ERROR_KEY);
  }
  if(!HAL_GPIO_ReadPin(port, pin)) {
	  return(KEYPAD_ERROR_KEY);
  }

  HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_RESET);
  if(!HAL_GPIO_ReadPin(port, pin)) {
	  column = 1;
  }
  HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_RESET);
  if(!HAL_GPIO_ReadPin(port, pin)) {
	  column = 2;
  }
  HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_RESET);
  if(!HAL_GPIO_ReadPin(port, pin)) {
	  column = 3;
  }
  HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_RESET);
  if(!HAL_GPIO_ReadPin(port, pin)) {
	  column = 4;
  }
  HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_SET);
  button = column + row * 4;
	/*
	if(button == 10) { // asterisk
		button = 11;
	} else if (button == 11) { // zero
		button = 0;
	} else if (button == 12) { // hash
		button = 12;
	}*/
  return(button);
}

char getKeyAsChar(uint16_t GPIO_Pin)
{
	int key = getKeyAsInt(GPIO_Pin);
	switch(key) {
	case KEYPAD_ERROR_KEY:
	  return(KEYPAD_ERROR_KEY & 0xFF);
	case 11:
      return('*');
	case 12:
	  return('#');
	default:
	  return('\0' + key);
	}
}
