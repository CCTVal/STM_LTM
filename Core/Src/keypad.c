/*
 * keypad.c
 *
 *  Created on: Jun 21, 2024
 *      Author: vsaona
 */

#include "keypad.h"

uint32_t previousMillis = 0;
uint64_t debouncing_time = 150;

typedef enum {
	IDLE		= 0x00,
	PUSH		= 0x01,
	HOLD		= 0x02,
	LIFT		= 0x03,
} keypad_state_t;

void keypad_Init()
{
	HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_SET);
}

uint8_t isKeyPressed()
{
	return(   HAL_GPIO_ReadPin(keypadRow1_GPIO_Port, keypadRow1_Pin)
			||HAL_GPIO_ReadPin(keypadRow2_GPIO_Port, keypadRow2_Pin)
			||HAL_GPIO_ReadPin(keypadRow3_GPIO_Port, keypadRow3_Pin)
			||HAL_GPIO_ReadPin(keypadRow4_GPIO_Port, keypadRow4_Pin));
}

uint16_t getPressedKey()
{
	uint16_t row, column;
	row = (KEYPAD_ERROR_KEY & 0x00F0) >> 2;
	column = KEYPAD_ERROR_KEY & 0x000F;
	uint16_t button = KEYPAD_ERROR_KEY;
	uint32_t currentMillis;
	GPIO_TypeDef* port;
	uint16_t GPIO_Pin;
	if(HAL_GPIO_ReadPin(keypadRow1_GPIO_Port, keypadRow1_Pin)) {
		GPIO_Pin = keypadRow1_Pin;
		row = 0;
		port = keypadRow1_GPIO_Port;
	} else if(HAL_GPIO_ReadPin(keypadRow2_GPIO_Port, keypadRow2_Pin)) {
		GPIO_Pin = keypadRow2_Pin;
		row = 0;
		port = keypadRow2_GPIO_Port;
	} else if(HAL_GPIO_ReadPin(keypadRow3_GPIO_Port, keypadRow3_Pin)) {
		GPIO_Pin = keypadRow3_Pin;
		row = 0;
		port = keypadRow3_GPIO_Port;
	} else if(HAL_GPIO_ReadPin(keypadRow4_GPIO_Port, keypadRow4_Pin)) {
		GPIO_Pin = keypadRow4_Pin;
		row = 0;
		port = keypadRow4_GPIO_Port;
	} else {
		return(KEYPAD_ERROR_KEY);
	}

	currentMillis = HAL_GetTick();
	if (currentMillis - previousMillis > debouncing_time) {
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			return(KEYPAD_ERROR_KEY);
		}
		HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 0;
		}
		HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 1;
		}
		HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 2;
		}
		HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 3;
		}
		HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_SET);
		button = column + row * 4;
	}
	previousMillis = currentMillis;
	return(button);
}

uint16_t getKeyAsInt(uint16_t GPIO_Pin)
{
	uint16_t row, column;
	row = (KEYPAD_ERROR_KEY & 0x00F0) >> 2;
	column = KEYPAD_ERROR_KEY & 0x000F;
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
	if (currentMillis - previousMillis > debouncing_time) {
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			return(KEYPAD_ERROR_KEY);
		}
		HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 0;
		}
		HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 1;
		}
		HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 2;
		}
		HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_RESET);
		if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
			column = 3;
		}
		HAL_GPIO_WritePin(keypadColumn4_GPIO_Port, keypadColumn4_Pin, GPIO_PIN_SET);
		button = column + row * 4;
	}
	previousMillis = currentMillis;
	return(button);
}

char getKeyAsChar(uint16_t GPIO_Pin)
{
	int key = getKeyAsInt(GPIO_Pin);
	switch(key) {
	case KEYPAD_ERROR_KEY:
		return(KEYPAD_ERROR_KEY);
	case 11:
		return('*');
	case 12:
		return('#');
	default:
		return('\0' + key);
	}
}

uint16_t checkKeypad()
{
	static keypad_state_t keypad_state = IDLE;
	static uint64_t debounce_time = 0;
	static uint16_t trying_key = KEYPAD_ERROR_KEY;
	static uint64_t initial_time = 0;
	uint16_t pressed_key = KEYPAD_ERROR_KEY;
	switch(keypad_state) {
	case IDLE:
		if(isKeyPressed()) {
			keypad_state = PUSH;
			trying_key = getPressedKey();
			initial_time = HAL_GetTick();
		}
	case PUSH:
		pressed_key = getPressedKey();
		if(isKeyPressed() && HAL_GetTick() - initial_time > debouncing_time && trying_key == pressedKey) {
			keypad_state = HOLD;
			return(trying_key);
		} else if(trying_key != pressedKey) {
			keypad_state = IDLE;
		}
	case HOLD:
		if(!isKeyPressed()) {
			// TODO
		}
	return(KEYPAD_ERROR_KEY);
}
