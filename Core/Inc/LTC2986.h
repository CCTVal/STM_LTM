/*
 * LTC2986.h
 *
 *  Created on: Jul 3, 2024
 *      Author: vsaona
 */

#ifndef INC_LTC2986_H_
#define INC_LTC2986_H_

#include "stm32f4xx_hal.h"

//**********************************************************************************************************
// -- STATUS BYTE CONSTANTS --
//**********************************************************************************************************
#define LTC2986_CONFIGURATION_ERROR (uint8_t) 0xFF
#define LTC2986_SENSOR_HARD_FAILURE (uint8_t) 0x80
#define LTC2986_ADC_HARD_FAILURE    (uint8_t) 0x40
#define LTC2986_CJ_HARD_FAILURE     (uint8_t) 0x20
#define LTC2986_CJ_SOFT_FAILURE     (uint8_t) 0x10
#define LTC2986_SENSOR_ABOVE        (uint8_t) 0x08
#define LTC2986_SENSOR_BELOW        (uint8_t) 0x04
#define LTC2986_ADC_RANGE_ERROR     (uint8_t) 0x02
#define LTC2986_VALID               (uint8_t) 0x01 // up when there is no error

//**************************************************************
// -- CONFIGURATION CONSTANTS
//**************************************************************
#define LTC2986_CELSIUS                  (uint8_t) 0x00
#define LTC2986_FAHRENHEIT               (uint8_t) 0x04
#define LTC2986_FILTER_55_HZ             (uint8_t) 0x00
#define LTC2986_FILTER_60_HZ             (uint8_t) 0x01
#define LTC2986_FILTER_50_HZ             (uint8_t) 0x02
#define LTC2986_3_WIRE_KELVIN            (uint8_t) 0x10
#define LTC2986_2_WIRE_KELVIN            (uint8_t) 0x20
#define LTC2986_THERMISTOR_KELVIN        (uint8_t) 0x40

typedef enum {
	LTC2986_TYPE_J_THERMOCOUPLE = 0x01,
	LTC2986_TYPE_K_THERMOCOUPLE = 0x02,
	LTC2986_TYPE_E_THERMOCOUPLE = 0x03,
	LTC2986_TYPE_N_THERMOCOUPLE = 0x04,
	LTC2986_TYPE_R_THERMOCOUPLE = 0x05,
	LTC2986_TYPE_S_THERMOCOUPLE = 0x06,
	LTC2986_TYPE_T_THERMOCOUPLE = 0x07,
	LTC2986_TYPE_B_THERMOCOUPLE = 0x08,
	LTC2986_CUSTOM_TYPE_THERMOCOUPLE = 0x09,
	LTC2986_RTD_PT_10 = 0x0A,
	LTC2986_RTD_PT_50 = 0x0B,
	LTC2986_RTD_PT_100 = 0x0C,
	LTC2986_RTD_PT_200 = 0x0D,
	LTC2986_RTD_PT_500 = 0x0E,
	LTC2986_RTD_PT_1000 = 0x0F,
	LTC2986_SENSE_RESISTOR = 0x1D,
	LTC2986_DIRECT_ADC = 0x1D,
	LTC2986_ANALOG_TEMPERATURE_SENSOR = 0x1D
} LTC2986_sensor_t;

typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
} LTC2986_gpio_t;

typedef struct {
    SPI_HandleTypeDef *spi_handle; // SCK = 5MHz
    LTC2986_gpio_t cs_pin;
} LTC2986_t;


void LTC2986_global_configure(LTC2986_t *LTM);
void LTC2986_configure_rtd(LTC2986_t *LTM, LTC2986_sensor_t type, uint8_t channel_number, uint8_t sense_channel);
void LTC2986_configure_sense_resistor(LTC2986_t *LTM, uint8_t channel_number, float resistance);
void LTC2986_configure_thermocouple(LTC2986_t *LTM, LTC2986_sensor_t type, uint8_t channel_number, uint8_t cold_juntion_channel);
float LTC2986_measure_channel(LTC2986_t *LTM, uint8_t channel_number);
uint8_t LTC2986_is_ready(LTC2986_t *LTM);
uint8_t LTC2986_read_status(LTC2986_t *LTM);



#endif /* INC_LTC2986_H_ */
