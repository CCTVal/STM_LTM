/*
 * LTC2986.c
 *
 *  Created on: Jul 3, 2024
 *      Author: vsaona
 */

#include "LTC2986.h"

#define LTC2986_SPI_TIMEOUT 50U

//************************************
// -- INSTRUCTIONS --
//************************************
#define WRITE_TO_RAM            (uint8_t) 0x02
#define READ_FROM_RAM           (uint8_t) 0x03
#define START_CONVERTION        (uint8_t) 0x80
//**********************************************************************************************************
// -- ADDRESSES --
//**********************************************************************************************************
#define LTC2986_COMMAND_STATUS_REGISTER          (uint16_t) 0x0000
#define LTC2986_STATUS_REGISTER                  (uint16_t) 0x0000
#define LTC2986_COMMAND_REGISTER                 (uint16_t) 0x0000
#define LTC2986_CH_ADDRESS_BASE                  (uint16_t) 0x0200
#define LTC2986_VOUT_CH_BASE                     (uint16_t) 0x0060
#define LTC2986_READ_CH_BASE                     (uint16_t) 0x0010
#define LTC2986_CONVERSION_RESULT_REGISTER       (uint16_t) 0x0010
#define LTC2986_GLOBAL_CONFIGURATION_REGISTER    (uint16_t) 0x00F0
#define LTC2986_DELAY_REGISTER                   (uint16_t) 0x00FF

/**
  * @brief  moves a 32 bit datuum to a 4 slots 8-bit array, for communication purposes.
  * @param  32-bit datuum
  * @param  8-bit data array. It is assummed to have size >= 4.
  */
void int32_to_int8_array(int32_t int32_value, uint8_t *int8_array)
{
    // Using pointer arithmetic to access each byte of the 32-bit integer
    *(int8_array + 3) = (uint8_t)(int32_value & 0xFF);        // Most significant byte (LSB)
    *(int8_array + 2) = (uint8_t)((int32_value >> 8) & 0xFF);
    *(int8_array + 1) = (uint8_t)((int32_value >> 16) & 0xFF);
    *(int8_array + 0) = (uint8_t)((int32_value >> 24) & 0xFF);  // Least significant byte (MSB)
    return;
}

/**
  * @brief  Read a specific register from LTM RAM.
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  address memory address in LTM's RAM memory to be accessed.
  * @param  length Register size in bytes. Typically it's 1, or 4
  * @param  buffer Pointer to where the data will be returned.
  */
void read_RAM(LTC2986_t *LTM, uint16_t address, int length, uint8_t *buffer)
{
	uint8_t read_instruction = READ_FROM_RAM;
	uint8_t address_8bit[2];
	address_8bit[1] = (uint8_t) (address) & 0xFF;
	address_8bit[0] = (uint8_t) (address >> 8) & 0xFF;
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(LTM->spi_handle, &read_instruction, 1, LTC2986_SPI_TIMEOUT); // Read instruction
	HAL_SPI_Transmit(LTM->spi_handle, address_8bit, 2, LTC2986_SPI_TIMEOUT); // Address
	HAL_SPI_Receive(LTM->spi_handle, buffer, length, LTC2986_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_SET);
	return;
}

/**
  * @brief  Write into a specific register from LTM RAM.
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  address memory address in LTM's RAM memory to be accessed.
  * @param  length data length in bytes to be sent. Should be equal to register size in bytes. Typically it's 1, or 4.
  * @param  buffer Pointer to the data that will be sent.
  */
void write_RAM(LTC2986_t *LTM, uint16_t address, int length, uint8_t *buffer)
{
	uint8_t write_instruction = WRITE_TO_RAM;
	uint8_t address_8bit[2];
	address_8bit[1] = (uint8_t) (address) & 0xFF;
	address_8bit[0] = (uint8_t) (address >> 8) & 0xFF;
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(LTM->spi_handle, &write_instruction, 1, LTC2986_SPI_TIMEOUT); // Read instruction
	HAL_SPI_Transmit(LTM->spi_handle, address_8bit, 2, LTC2986_SPI_TIMEOUT); // Address
	HAL_SPI_Transmit(LTM->spi_handle, buffer, length, LTC2986_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LTM->cs_pin.gpio_port, LTM->cs_pin.gpio_pin, GPIO_PIN_SET);
	return;
}

/**
  * @brief  Write global configuration for LTM, in both "global configuration"
  *               register and "mux extra configuration setting time" register.
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  */
void LTC2986_global_configure(LTC2986_t *LTM)
{
	uint8_t config = LTC2986_CELSIUS | LTC2986_FILTER_50_HZ; // no excitation mode
	write_RAM(LTM, LTC2986_GLOBAL_CONFIGURATION_REGISTER, 1, &config);
	uint8_t extra_delay = 0; // Add 1 per each micro-second desired. Zero equals 1 millisecond for configuration setting.
	write_RAM(LTM, LTC2986_DELAY_REGISTER, 1, &extra_delay);
	return;
}

// Esta configuracion siento que tiene demasiados parametros asi que por ahora la voy a dejar hardcodeada, pero si queremos hacer una biblioteca portable y general, se deberian dejar como paramteros.
/**
  * @brief  Configure a channel for an RTD sensor
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  type Type of RTD (PT-10, 50, 100, 200, 500, 1000, RTD-1000 or NI-120)
  * @param  channel_number To which channel the RTD is to be connected
  * @param  sense_channel Associated sense resistor channel.
  */
void LTC2986_configure_rtd(LTC2986_t *LTM, LTC2986_sensor_t type, uint8_t channel_number, uint8_t sense_channel)
{
	uint32_t configuration;
	configuration = type << 27;
	configuration |= (sense_channel) << 22;
	configuration |= 0x0 << 20; // 2-wire
	configuration |= 0x1 << 18; // no rotation, sharing
	configuration |= 0x7 << 14; // 500 uA
	configuration |= 0x1 << 12; // American curve for the RTD.
	uint8_t temp[4];
	int32_to_int8_array(configuration, temp);
	write_RAM(LTM, LTC2986_CH_ADDRESS_BASE + (4 * (channel_number - 1)), 4, temp);
	return;
}

/**
  * @brief  Configure a channel for a thermocouple
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  type Type of thermocouple (J, K, E, N, R, S, T, B)
  * @param  channel_number To which channel the thermocouple is to be connected
  * @param  cold_junction_channel Associated cold junction temperature sensor channel.
  */
void LTC2986_configure_thermocouple(LTC2986_t *LTM, LTC2986_sensor_t type, uint8_t channel_number, uint8_t cold_junction_channel)
{
	uint32_t configuration;
	configuration = type << 27;
	configuration |= cold_junction_channel << 22;
	configuration |= 0b1 << 21; // Single-ended
	configuration |= 0b0 << 20; // TC_OPEN_CKT_DETECT__NO
	configuration |= 0x0 << 18; // TC_OPEN_CKT_DETECT_CURRENT__10UA
	uint8_t temp[4];
	int32_to_int8_array(configuration, temp);
	write_RAM(LTM, LTC2986_CH_ADDRESS_BASE + (4 * (channel_number - 1)), 4, temp);
	return;
}

/**
  * @brief  Configure a channel for a sense resistor
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  channel_number To which channel the thermocouple is to be connected
  * @param  resistance Sense resistor value, up to 131 Ohm. There is a 1/1024 resolution.
  */
void LTC2986_configure_sense_resistor(LTC2986_t *LTM, uint8_t channel_number, float resistance)
{
	uint32_t configuration = 29 << 27;
	if(resistance > 131071) {
		// This is a programming error.
		resistance = 131071;
	}
	configuration |= (uint32_t) ((int) resistance* 1024);
	uint8_t temp[4];
	int32_to_int8_array(configuration, temp);
	write_RAM(LTM, LTC2986_CH_ADDRESS_BASE + (4 * (channel_number - 1)), 4, temp);
	return;
}

/**
  * @brief  Check status register and answer if it is ready or not.
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @retval  Boolean Whether the chip is ready for new orders.
  */
uint8_t LTC2986_is_ready(LTC2986_t *LTM)
{
	uint8_t status;
	read_RAM(LTM, LTC2986_STATUS_REGISTER, 1, &status);
	uint8_t started_process = !!(status & START_CONVERTION);
	uint8_t done = !!(status & 0x40);
	return(done && !started_process);
}

/**
  * @brief  Read LTC2986 status register
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @retval  Status register current value.
  */
uint8_t LTC2986_read_status(LTC2986_t *LTM)
{
	uint8_t status;
	read_RAM(LTM, LTC2986_STATUS_REGISTER, 1, &status);
	return(status);
}

/**
  * @brief  Requests a channel to be measured, and waits for the convertion to finish
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  channel_number to be read.
  * @retval  Floating point in Celsius (or Fahrenheit, according to configuration) read by LTM chip.
  */
float LTC2986_measure_channel(LTC2986_t *LTM, uint8_t channel_number)
{
	uint8_t convert_instruction = channel_number | START_CONVERTION;
	write_RAM(LTM, LTC2986_COMMAND_REGISTER, 1, &convert_instruction); // We command to initiate the conversion
	HAL_Delay(10);
	while(!LTC2986_is_ready(LTM));
	uint8_t temp[4];
	uint16_t address = LTC2986_CONVERSION_RESULT_REGISTER + (4 * (channel_number - 1));
	read_RAM(LTM, address, 4, temp);
	uint8_t fault = temp[0];
	if(fault != LTC2986_VALID) {
		uint8_t return_value[4] = {0xFF, 0xFF, 0xFF, fault};
		float *casted_return_value;
		casted_return_value = (float*) return_value;
		return(*casted_return_value); // Return a NaN with the fault encoded
	}
	uint32_t raw_result = (((uint32_t) temp[3]) | ((uint32_t) temp[2] << 8) | ((uint32_t) temp[1] << 16)) & 0x00FFFFFF;
	float result = ((float) raw_result) / 1024; // Assuming it is a temperature channel (not voltage, for example)
	return(result);
}

/**
  * @brief  Requests a channel to be measured
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  channel_number to be read.
  * @retval  Floating point in Celsius (or Fahrenheit, according to configuration) read by LTM chip.
  */
void LTC2986_init_measurement(LTC2986_t *LTM, uint8_t channel_number)
{
	uint8_t convert_instruction = channel_number | START_CONVERTION;
	write_RAM(LTM, LTC2986_COMMAND_REGISTER, 1, &convert_instruction); // We command to initiate the conversion
	return;
}

/**
  * @brief  Waits for a previously requested convertion to finish and retrieves the value
  * @param  LTM pointer to a LTC2986 structure that contains
  *               the configuration information for SPI module.
  * @param  channel_number to be read.
  * @retval  Floating point in Celsius (or Fahrenheit, according to configuration) read by LTM chip.
  */
float LTC2986_fetch_measurement(LTC2986_t *LTM, uint8_t channel_number)
{
	while(!LTC2986_is_ready(LTM));
	uint8_t temp[4];
	uint16_t address = LTC2986_CONVERSION_RESULT_REGISTER + (4 * (channel_number - 1));
	read_RAM(LTM, address, 4, temp);
	uint8_t fault = temp[0];
	if(fault != LTC2986_VALID) {
		uint8_t return_value[4] = {0xFF, 0xFF, 0xFF, fault};
		float *casted_return_value;
		casted_return_value = (float*) return_value;
		return(*casted_return_value); // Return a NaN with the fault encoded
	}
	uint32_t raw_result = (((uint32_t) temp[3]) | ((uint32_t) temp[2] << 8) | ((uint32_t) temp[1] << 16)) & 0x00FFFFFF;
	float result = ((float) raw_result) / 1024; // Assuming it is a temperature channel (not voltage, for example)
	return(result);
}

