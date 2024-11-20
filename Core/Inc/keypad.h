/*
 * keypad.h
 *
 *  Created on: June 21, 2024
 *      Author: vsaona
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "main.h"

#define KEYPAD_ERROR_KEY	0x000A

void keypad_Init();
uint16_t getKeyAsInt(uint16_t GPIO_Pin);
char getKeyAsChar(uint16_t GPIO_Pin);

#endif /* KEYPAD_H_ */
