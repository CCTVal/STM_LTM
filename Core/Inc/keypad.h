/*
 * keypad.h
 *
 *  Created on: June 21, 2024
 *      Author: vsaona
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "main.h"

#define KEYPAD_ERROR_KEY	0x00FF
#define KEYPAD_A_KEY		0x000A
#define KEYPAD_CALIB_KEY	0x000A
#define KEYPAD_B_KEY		0x000B
#define KEYPAD_YES_KEY		0x000B
#define KEYPAD_C_KEY		0x000C
#define KEYPAD_NO_KEY		0x000C
#define KEYPAD_D_KEY		0x000D
#define KEYPAD_E_KEY		0x000E
#define KEYPAD_EXIT_KEY		0x000E


void keypad_Init();
uint16_t getKeyAsInt(uint16_t GPIO_Pin);
char getKeyAsChar(uint16_t GPIO_Pin);
uint16_t checkKeypad();

#endif /* KEYPAD_H_ */
