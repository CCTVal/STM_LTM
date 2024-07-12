/*
 * max7219.c
 *
 *  Created on: May 11, 2019
 *      Author: tabur
 */

#include "max7219.h"

#define CS_SET() 	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET)
#define CS_RESET() 	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET)

static uint8_t decodeMode = 0x00;

static uint8_t SYMBOLS[] = {
		0x7E,	// numeric 0
		0x30,	// numeric 1
		0x6D,	// numeric 2
		0x79,	// numeric 3
		0x33,	// numeric 4
		0x5B,	// numeric 5
		0x5F,	// numeric 6
		0x70,	// numeric 7
		0x7F,	// numeric 8
		0x7B,	// numeric 9
		0x01,	// minus
		0x4F,	// letter E
		0x37,	// letter H
		0x0E,	// letter L
		0x67,	// letter P
		0x00	// blank
};

static uint16_t getSymbol(uint8_t number);
static uint32_t lcdPow10(uint8_t n);

void max7219_Init(uint8_t intensivity)
{
	max7219_Turn_On();
	for (int chip = 1; chip <= NUMBER_OF_CHIPS; chip++)
	{
		max7219_SendData(REG_SCAN_LIMIT, NUMBER_OF_DIGITS - 1, chip);
	}
	max7219_SetIntensivity(intensivity);
	max7219_Clean();
}

void max7219_SetIntensivity(uint8_t intensivity)
{
	if (intensivity > 0x0F)
	{
		return;
	}
	for (int chip = 1; chip <= NUMBER_OF_CHIPS; chip++)
	{
		max7219_SendData(REG_INTENSITY, intensivity, chip);
	}
}

void max7219_Clean()
{
	uint8_t clear = 0x00;

	if(decodeMode == 0xFF)
	{
		clear = BLANK;
	}

	for (int chip = 1; chip <= NUMBER_OF_CHIPS; chip++)
	{
		for (int i = 0; i < 8; ++i)
		{
			max7219_SendData(i + 1, clear, chip);
		}
	}
}

void max7219_SendData(uint8_t addr, uint8_t data, uint8_t chip)
{
	CS_SET();
	uint8_t no_op_address = REG_NO_OP;
	for(int i = 0; i < NUMBER_OF_CHIPS - chip; i++) {
		HAL_SPI_Transmit(&hspi3, &no_op_address, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi3, &no_op_address, 1, HAL_MAX_DELAY);
	}
	HAL_SPI_Transmit(&hspi3, &addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &data, 1, HAL_MAX_DELAY);
	for(int i = 1; i < chip; i++) {
		HAL_SPI_Transmit(&hspi3, &no_op_address, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi3, &no_op_address, 1, HAL_MAX_DELAY);
	}
	CS_RESET();
}

void max7219_Turn_On(void)
{
	for (int chip = 1; chip <= NUMBER_OF_CHIPS; chip++)
	{
		max7219_SendData(REG_SHUTDOWN, 0x01, chip);
	}
}

void max7219_Turn_Off(void)
{
	for (int chip = 1; chip <= NUMBER_OF_CHIPS; chip++)
	{
		max7219_SendData(REG_SHUTDOWN, 0x00, chip);
	}
}

void max7219_Decode_On(void)
{
	decodeMode = 0xFF;
	for (int chip = 1; chip <= NUMBER_OF_CHIPS; chip++)
	{
		max7219_SendData(REG_DECODE_MODE, decodeMode, chip);
	}
}

void max7219_Decode_Off(void)
{
	decodeMode = 0x00;
	for (int chip = 1; chip <= NUMBER_OF_CHIPS; chip++)
	{
		max7219_SendData(REG_DECODE_MODE, decodeMode, chip);
	}
}

void max7219_PrintDigit(MAX7219_Digits position, MAX7219_Numeric numeric, bool point)
{
	if(position > NUMBER_OF_DIGITS * NUMBER_OF_CHIPS)
	{
		return;
	}

	int chip = (position - 1) / NUMBER_OF_DIGITS + 1;
	position = (position - 1) % NUMBER_OF_DIGITS + 1;

	if(point)
	{
		if(decodeMode == 0x00)
		{
			max7219_SendData(position, getSymbol(numeric) | (1 << 7), chip);
		}
		else if(decodeMode == 0xFF)
		{
			max7219_SendData(position, numeric | (1 << 7), chip);
		}
	}
	else
	{
		if(decodeMode == 0x00)
		{
			max7219_SendData(position, getSymbol(numeric) & (~(1 << 7)), chip);
		}
		else if(decodeMode == 0xFF)
		{
			max7219_SendData(position, numeric & (~(1 << 7)), chip);
		}
	}
}

MAX7219_Digits max7219_PrintItos(MAX7219_Digits position, int value)
{
	int chip = (position - 1) / NUMBER_OF_DIGITS + 1;
	position = (position - 1) % NUMBER_OF_DIGITS + 1;
	max7219_SendData(REG_DECODE_MODE, 0xFF, chip);

	int32_t i;

	if (value < 0)
	{
		if(position > 0)
		{
			max7219_SendData(position, MINUS, chip);
			position--;
		}
		value = -value;
	}

	i = 1;

	while ((value / i) > 9)
	{
		i *= 10;
	}

	if(position > 0)
	{
		max7219_SendData(position, value/i, chip);
		position--;
	}

	i /= 10;

	while (i > 0)
	{
		if(position > 0)
		{
			max7219_SendData(position, (value % (i * 10)) / i, chip);
			position--;
		}

		i /= 10;
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode, chip);

	return position;
}

MAX7219_Digits max7219_PrintNtos(MAX7219_Digits position, uint32_t value, uint8_t n)
{
	int chip = (position - 1) / NUMBER_OF_DIGITS + 1;
	position = (position - 1) % NUMBER_OF_DIGITS + 1;
	max7219_SendData(REG_DECODE_MODE, 0xFF, chip);

	if (n > 0u)
	{
		uint32_t i = lcdPow10(n - 1u);

		while (i > 0u)	/* Display at least one symbol */
		{
			if(position > 0u)
			{
				max7219_SendData(position, (value / i) % 10u, chip);
				position--;
			}

			i /= 10u;
		}
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode, chip);

	return position;
}

MAX7219_Digits max7219_PrintFtos(MAX7219_Digits position, float value, uint8_t n)
{
	if(n > 4)
	{
		n = 4;
	}

	int chip = (position - 1) / NUMBER_OF_DIGITS + 1;
	position = (position - 1) % NUMBER_OF_DIGITS + 1;
	max7219_SendData(REG_DECODE_MODE, 0xFF, chip);

	if (value < 0.0)
	{
		if(position > 0)
		{
			max7219_SendData(position, MINUS, chip);
			position--;
		}

		value = -value;
	}

	position = max7219_PrintItos(position + ((chip - 1) * NUMBER_OF_DIGITS), (int32_t) value);

	if (n > 0u)
	{
		max7219_PrintDigit(position + 1 + ((chip - 1) * NUMBER_OF_DIGITS), ((int32_t) value) % 10, true);

		position = max7219_PrintNtos(position + ((chip - 1) * NUMBER_OF_DIGITS), (uint32_t) (value * (float) lcdPow10(n)), n);
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode, chip);

	return position;
}

static uint16_t getSymbol(uint8_t number)
{
	return SYMBOLS[number];
}

static uint32_t lcdPow10(uint8_t n)
{
	uint32_t retval = 1u;

	while (n > 0u)
	{
		retval *= 10u;
		n--;
	}

	return retval;
}
